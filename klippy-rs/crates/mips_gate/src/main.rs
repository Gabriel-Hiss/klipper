//! Exercises the platform paths the port depends on, printing results that are compared
//! between x86_64 and the KE target: Python float formatting, libm through `kos`, C casts,
//! threads with condvars, and reactor timers.

fn main() {
    // Python float text and libm.
    for x in [0.1, 1.0 / 3.0, 1e16, 1e-5, -0.0, 123456.789] {
        println!("repr {}", pycompat::text::float_repr(x));
    }
    for x in [0.5f64, 1.25, 3.0] {
        println!(
            "libm exp={} sin={} pow={}",
            pycompat::text::float_repr(kos::libm::exp(x)),
            pycompat::text::float_repr(kos::libm::sin(x)),
            pycompat::text::float_repr(kos::libm::pow(x, 2.5))
        );
    }
    // Threads and sync.
    let counter = std::sync::Arc::new(kos::sync::Mutex::new(0u32));
    let mut handles = Vec::new();
    for _ in 0..4 {
        let c = counter.clone();
        handles.push(kos::thread::spawn(move || {
            for _ in 0..1000 {
                *c.lock() += 1;
            }
        }));
    }
    for h in handles {
        h.join();
    }
    println!("threads counter={}", *counter.lock());
    // Reactor timers.
    let r = reactor::Reactor::new(false);
    let hits = std::sync::Arc::new(kos::sync::Mutex::new(0u32));
    let h2 = hits.clone();
    let r2 = r.clone();
    r.register_timer(
        move |eventtime| {
            let mut n = h2.lock();
            *n += 1;
            if *n == 3 {
                r2.end();
                return reactor::NEVER;
            }
            eventtime + 0.01
        },
        reactor::NOW,
    );
    r.run();
    println!("reactor timer hits={}", *hits.lock());
}
