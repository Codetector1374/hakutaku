pub fn main() {
    println!("cargo:rerun-if-env-changed=VERBOSE_BUILD");
}
