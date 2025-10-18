use hackrf_sys::hackrf_init;

fn main() {
    let rv = unsafe { hackrf_init() };
    println!("hackrf_init returned {rv}");
}
