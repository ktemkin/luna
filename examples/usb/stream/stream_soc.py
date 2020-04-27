#!/usr/bin/env python3
#
# This file is part of LUNA.
#

from nmigen                                  import Elaboratable, Module, Cat, Signal
from nmigen.hdl.rec                          import Record
from nmigen.hdl.mem                          import Memory

from usb_protocol.emitters                   import DeviceDescriptorCollection

from lambdasoc.periph.serial                 import AsyncSerialPeripheral
from lambdasoc.periph.timer                  import TimerPeripheral

from luna                                    import top_level_cli
from luna.gateware.soc                       import SimpleSoC
from luna.gateware.architecture.car          import LunaECP5DomainGenerator

from luna.gateware.usb.usb2.device           import USBDevice, USBDeviceController
from luna.gateware.usb.usb2.endpoint         import EndpointInterface

from luna.gateware.soc.peripheral            import Peripheral


CLOCK_FREQUENCIES_MHZ = {
    'sync': 60
}


class CPUControlledBulkIn(Peripheral, Elaboratable):

    #
    # Registers:
    #    data    <-- fifo
    #    send    <-- write '1' to send
    #    pending <-- reads '1' iff a packet is pending
    #

    def __init__(self, *, endpoint_number, max_packet_size):
        super().__init__()

        self._endpoint_number = endpoint_number
        self._max_packet_size = max_packet_size

        #
        # I/O port
        #
        self.interface = EndpointInterface()

        #
        # Registers
        #

        regs = self.csr_bank()
        self._data    = regs.csr(8, "w", desc="Writes to this register place data into our transmit FIFO.")
        self._send    = regs.csr(1, "w", desc="Writes to this register prime our endpoint to send the relevant data.")
        self._pending = regs.csr(1, "r", desc="Contains `1` iff a transmission is scheduled on this endpoint.")

        # ... and convert our register into a Wishbone peripheral.
        self._bridge    = self.bridge(data_width=32, granularity=8, alignment=2)
        self.bus        = self._bridge.bus


    def elaborate(self, platform):
        m = Module()
        m.submodules.bridge = self._bridge

        #
        # Packet buffer
        #
        buffer = Memory(width=8, depth=self._max_packet_size)
        m.submodules.buffer_write = buffer_write = buffer.write_port()
        m.submodules.buffer_read  = buffer_read  = buffer.read_port()

        # Count that keeps track of how many bytes are in our packet buffer;
        # and where we are in sending data from that buffer
        data_count     = Signal(range(0, self._max_packet_size + 1))
        send_position  = Signal.like(data_count)

        buffer_empty   = (data_count == 0)
        buffer_full    = (data_count == self._max_packet_size)

        # Increment our counter each time the host writes data into our buffer.
        with m.If(self._data.w_stb & ~buffer_full):
            m.d.sync += data_count.eq(data_count + 1)


        #
        # Data queueing logic.
        #

        tokenizer         = self.interface.tokenizer
        handshakes_out    = self.interface.handshakes_in

        tx                = self.interface.tx
        tx_pid            = self.interface.tx_pid_toggle

        endpoint_matching = (tokenizer.endpoint == self._endpoint_number)
        in_token_received = endpoint_matching & tokenizer.is_in & tokenizer.ready_for_response

        m.d.comb += [

            # Interface for filling our memory.
            buffer_write.addr   .eq(data_count),
            buffer_write.data   .eq(self._data.w_data),
            buffer_write.en     .eq(self._data.w_stb & ~buffer_full)
        ]

        #
        # Transmit logic.
        #
        with m.FSM():

            # WAIT_FOR_DATA -- we're waiting for the CPU to provide us with
            # a send command
            with m.State("WAIT_FOR_DATA"):
                m.d.comb += handshakes_out.nak  .eq(in_token_received)

                # Once we have our send command, wait for an IN token from the host.
                with m.If(self._send.w_stb & self._send.w_data):
                    m.next = "WAIT_FOR_IN_TOKEN"
                    m.d.sync += send_position.eq(0)


            # WAIT_FOR_IN -- wait for our host to send us an IN token
            with m.State("WAIT_FOR_IN_TOKEN"):

                # Once we get that IN token, move to sending our data.
                with m.If(in_token_received):
                    m.next = "SEND_PACKET"


            with m.State("SEND_PACKET"):

                last_byte  = (send_position + 1 == data_count) | buffer_empty
                first_byte = (send_position == 0) & ~last_byte

                m.d.comb += [
                    tx.valid             .eq(1),
                    tx.payload           .eq(buffer_read.data),

                    tx.first             .eq(first_byte),
                    tx.last              .eq(last_byte),

                    buffer_read.addr     .eq(send_position)
                ]

                # Every time our transmitter accepts a byte, move on to the next one.
                with m.If(tx.ready):
                    m.d.sync += send_position.eq(send_position + 1)

                    # When we'll have send as much as we need to, wait for an ACK.
                    with m.If(last_byte):
                        m.next = "WAIT_FOR_ACK"


            with m.State("WAIT_FOR_ACK"):

                # If the host ACKs our data, we're good to move to the next packet.
                with m.If(self.interface.handshakes_in.ack):
                    m.d.sync += [
                        tx_pid.eq(~tx_pid),
                        data_count.eq(0),
                    ]
                    m.next = "WAIT_FOR_DATA"

                # If we don't receive an ACK, re-transmit our data from the beginning.
                with m.If(tokenizer.new_token):
                    m.d.sync += send_position.eq(0)
                    m.next = "WAIT_FOR_IN_TOKEN"


        return m


class StreamSoCExample(Elaboratable):

    def create_descriptors(self):
        """ Create the descriptors we want to use for our device. """

        descriptors = DeviceDescriptorCollection()

        #
        # We'll add the major components of the descriptors we we want.
        # The collection we build here will be necessary to create a standard endpoint.
        #

        # We'll need a device descriptor...
        with descriptors.DeviceDescriptor() as d:
            d.idVendor           = 0x16d0
            d.idProduct          = 0xf3b

            d.iManufacturer      = "LUNA"
            d.iProduct           = "Stream Example Device"
            d.iSerialNumber      = "1234"

            d.bNumConfigurations = 1


        # ... and a description of the USB configuration we'll provide.
        with descriptors.ConfigurationDescriptor() as c:

            with c.InterfaceDescriptor() as i:
                i.bInterfaceNumber = 0

                with i.EndpointDescriptor() as e:
                    e.bEndpointAddress = 0x01
                    e.wMaxPacketSize   = 64

        return descriptors


    def __init__(self):

        # Create a stand-in for our UART.
        self.uart_pins = Record([
            ('rx', [('i', 1)]),
            ('tx', [('o', 1)])
        ])

        # Create our SoC...
        self.soc = soc = SimpleSoC()

        # ... add our firmware image ...
        soc.add_rom("stream.bin", 0x4000)

        # ... add some bulk RAM ...
        soc.add_ram(0x4000)

        # ... add a UART ...
        self.uart = uart = AsyncSerialPeripheral(divisor=int(60e6 // 115200), pins=self.uart_pins)
        soc.add_peripheral(uart)

        # ... a core USB controller ...
        self.controller = USBDeviceController()
        soc.add_peripheral(self.controller)

        # ... and a bulk endpoint.
        self.bulk_in = CPUControlledBulkIn(endpoint_number=0x01, max_packet_size=64)
        soc.add_peripheral(self.bulk_in, as_submodule=False)




    def elaborate(self, platform):
        m = Module()
        m.submodules.soc = self.soc

        # Generate our domain clocks/resets.
        m.submodules.car = LunaECP5DomainGenerator(clock_frequencies=CLOCK_FREQUENCIES_MHZ)

        # Connect up our UART.
        uart_io = platform.request("uart", 0)
        m.d.comb += [
            uart_io.tx         .eq(self.uart_pins.tx),
            self.uart_pins.rx  .eq(uart_io.rx)
        ]

        # Create our USB device.
        ulpi = platform.request("target_phy")
        m.submodules.usb = usb = USBDevice(bus=ulpi)
        usb.add_standard_control_endpoint(self.create_descriptors())
        usb.add_endpoint(self.bulk_in)


        # Connect up our device controller.
        m.d.comb += self.controller.attach(usb)

        return m


if __name__ == "__main__":
    design = StreamSoCExample()
    top_level_cli(design, cli_soc=design.soc)
