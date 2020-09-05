use super::UsbBus;
use heapless::{Vec, consts};

struct SerialTerminal {
    usb_device: usb_device::device::UsbDevice<'static, UsbBus>,
    usb_serial: usbd_serial::SerialPort<'static, UsbBus>,
    buffer: Vec<u8, consts::U128>,
}

impl SerialTerminal {
    pub fn new(
        usb_device: usb_device::device::UsbDevice<'static, UsbBus>,
        usb_serial: usbd_serial::SerialPort<'static, UsbBus>,
    ) -> Self {
        Self { usb_device, usb_serial }
    }

    pub fn poll(&self) -> Option<Request> {
        // Update the USB serial port.
        if self.usb_device.poll(&mut [&mut self.usb_serial]) == false {
            return None;
        }

        // Consume data from the serial port.
        let mut buffer = [0u8; 64];
        match self.usb_serial.read(&mut buffer) {
            Ok(count) => {
                match self.buffer.extend_from_slice(buffer[..count]) {
                    Err(_) => {
                        // Make a best effort to inform the user of the overflow.
                        self.usb_serial.write("\nBuffer overflow\n".as_bytes()).ok();
                        self.buffer.clear()
                }
        }

    }
}
