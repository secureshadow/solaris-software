#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use esp_hal::clock::CpuClock;
use esp_hal::main;
use esp_hal::time::{Rate};
use esp_hal::spi::{
    Mode,
    master::{Config, Spi},
};
use esp_hal::delay::Delay;
use esp_println::println;

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    // generator version: 0.5.0


    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);
    
    let sck = peripherals.GPIO48;
    let miso = peripherals.GPIO47;
    let mosi = peripherals.GPIO38;
    let cs_bmp = peripherals.GPIO18;

    let mut spi_bmp = Spi::new(
        peripherals.SPI2,
        Config::default()
            .with_frequency(Rate::from_khz(100))
            .with_mode(Mode::_0),
        )
        .unwrap()
        .with_sck(sck)
        .with_miso(miso)
        .with_mosi(mosi)
        .with_cs(cs_bmp);

    let delay = Delay::new();
    

    let mut data = [0x80,0x0]; // Read operation to register 0
    let result = spi_bmp.transfer(&mut data);
    match result {
        Ok(()) => {
            let chip_id = data[1];
            println!("BMP390 WHO_AM_I = 0x{:02X}", chip_id);
        }
        Err(e) => {
            println!("Error en la transferencia SPI: {:?}", e);
        }
    }
    delay.delay_millis(1000);
    loop{
        
    }

}
