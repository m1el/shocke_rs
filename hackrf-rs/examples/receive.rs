use core::ops::ControlFlow;
use hackrf_rs::{
    options::RxOptions,
    DeviceList, Hackrf,
};

type LazyResult<T> = Result<T, Box<dyn core::error::Error>>;

fn main() -> LazyResult<()> {
    println!("Available devices:");
    for device in DeviceList::new()?.iter() {
        println!("{device:?}");
    }
    let mut hackrf = Hackrf::open_first()?;
    let options = RxOptions {
        center_freq: 433_500_000_u64.into(),
        sample_rate: 10_000_000_f64.into(),
        bandwidth: 3_000_000,
        enable_amp: true,
        enable_bias_tee: true,
        lna_gain: 20,
        vga_gain: 20,
    };
    let mut data = Vec::new();
    let mut prev_checkpoint = 0;
    let threshold = 10_000_000;
    let (done_tx, done_rx) = std::sync::mpsc::sync_channel(1);
    hackrf.start_rx(options, move |samples| {
        data.extend_from_slice(samples);
        let total_samples = data.len() / 2;
        if prev_checkpoint + threshold < total_samples {
            let diff = (total_samples - prev_checkpoint) / threshold * threshold;
            prev_checkpoint += diff;
            println!("total received: {total_samples}");
        }
        if total_samples > 30_000_000 {
            let out = core::mem::take(&mut data);
            done_tx.send(out).unwrap();
            ControlFlow::Break(())
        } else {
            ControlFlow::Continue(())
        }
    })?;
    let data = done_rx.recv()?;
    std::fs::write("examples/received.bin", data)?;
    println!("done");
    hackrf.stop_rx()?;

    Ok(())
}