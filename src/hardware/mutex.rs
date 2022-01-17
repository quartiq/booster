/// A "mutex" that guarantees only a single user of a bus at a time through panicking.
pub struct AtomicCheckMutex<BUS> {
    bus: core::cell::UnsafeCell<BUS>,
    busy: core::sync::atomic::AtomicBool,
}

impl<BUS> shared_bus::BusMutex for AtomicCheckMutex<BUS> {
    type Bus = BUS;

    fn create(v: BUS) -> Self {
        Self {
            bus: core::cell::UnsafeCell::new(v),
            busy: core::sync::atomic::AtomicBool::from(false),
        }
    }

    fn lock<R, F: FnOnce(&mut Self::Bus) -> R>(&self, f: F) -> R {
        self.busy
            .compare_exchange(
                false,
                true,
                core::sync::atomic::Ordering::SeqCst,
                core::sync::atomic::Ordering::SeqCst,
            )
            .expect("Bus conflict");
        let result = f(unsafe { &mut *self.bus.get() });

        self.busy.store(false, core::sync::atomic::Ordering::SeqCst);

        result
    }
}

// It is explicitly safe to share this across threads because there is a coherency check using an
// atomic bool comparison.
unsafe impl<BUS> Sync for AtomicCheckMutex<BUS> {}

/// A convenience type for declaring use with shared-bus.
pub type AtomicCheckManager<T> = shared_bus::BusManager<AtomicCheckMutex<T>>;

/// Construct a statically allocated bus manager.
#[macro_export]
macro_rules! new_atomic_check_manager {
    ($bus_type:ty = $bus:expr) => {{
        let m: Option<&'static mut _> = cortex_m::singleton!(
            : $crate::hardware::mutex::AtomicCheckManager<$bus_type> =
                $crate::hardware::mutex::AtomicCheckManager::new($bus)
        );

        m
    }};
}
