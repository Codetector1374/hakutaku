
pub struct Serial16650 {
    use_mmio: bool,
    registers: &'static mut Serial16650Registers,
}

struct Serial16650Registers {

}