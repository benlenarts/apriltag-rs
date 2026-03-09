/// Parallelism strategy: sequential or parallel (rayon).
///
/// Encapsulates both compile-time feature gating and runtime thread-count
/// checking. When the `parallel` feature is enabled but only one rayon
/// thread is available, the sequential path is used to avoid overhead.
#[derive(Clone, Copy)]
pub(crate) enum Par {
    Sequential,
    #[cfg(feature = "parallel")]
    Parallel,
}

impl Par {
    /// Choose the execution strategy based on compile-time features and
    /// runtime thread count.
    pub(crate) fn get() -> Self {
        #[cfg(feature = "parallel")]
        if rayon::current_num_threads() > 1 {
            return Self::Parallel;
        }
        Self::Sequential
    }

    /// Process chunks of a mutable buffer with an indexed closure.
    ///
    /// Parallel: `par_chunks_mut` + `enumerate` + `for_each`.
    /// Sequential: plain `chunks_mut` loop.
    pub(crate) fn chunks_mut_for_each<T: Send>(
        self,
        buf: &mut [T],
        chunk_size: usize,
        f: impl Fn(usize, &mut [T]) + Send + Sync,
    ) {
        match self {
            Self::Sequential => {
                for (i, chunk) in buf.chunks_mut(chunk_size).enumerate() {
                    f(i, chunk);
                }
            }
            #[cfg(feature = "parallel")]
            Self::Parallel => {
                use rayon::prelude::*;
                buf.par_chunks_mut(chunk_size)
                    .enumerate()
                    .for_each(|(i, c)| f(i, c));
            }
        }
    }

    /// Map over a mutable slice with per-thread init, flatten `Option` results,
    /// and collect into a `Vec`.
    ///
    /// Parallel: `par_iter_mut` + `map_init` + `flatten` + `collect`.
    /// Sequential: single init, `filter_map` + `collect`.
    pub(crate) fn map_init_collect<T, B, R>(
        self,
        slice: &mut [T],
        init: impl Fn() -> B + Send + Sync,
        f: impl Fn(&mut B, &mut T) -> Option<R> + Send + Sync,
    ) -> Vec<R>
    where
        T: Send,
        B: Send,
        R: Send,
    {
        match self {
            Self::Sequential => {
                let mut bufs = init();
                slice
                    .iter_mut()
                    .filter_map(|item| f(&mut bufs, item))
                    .collect()
            }
            #[cfg(feature = "parallel")]
            Self::Parallel => {
                use rayon::prelude::*;
                slice
                    .par_iter_mut()
                    .map_init(init, |b, item| f(b, item))
                    .flatten()
                    .collect()
            }
        }
    }

    /// For-each over a mutable slice with per-thread init.
    ///
    /// Parallel: `par_iter_mut` + `for_each_init`.
    /// Sequential: single init, plain loop.
    pub(crate) fn for_each_init<T, B>(
        self,
        slice: &mut [T],
        init: impl Fn() -> B + Send + Sync,
        f: impl Fn(&mut B, &mut T) + Send + Sync,
    ) where
        T: Send,
        B: Send,
    {
        match self {
            Self::Sequential => {
                let mut bufs = init();
                for item in slice.iter_mut() {
                    f(&mut bufs, item);
                }
            }
            #[cfg(feature = "parallel")]
            Self::Parallel => {
                use rayon::prelude::*;
                slice
                    .par_iter_mut()
                    .for_each_init(init, |b, item| f(b, item));
            }
        }
    }

    /// Map over an immutable slice with per-thread init, appending results
    /// to a `Vec<R>` via a closure, then flatten and collect.
    ///
    /// Parallel: `par_iter` + `map_init` (with thread-local `Vec`) + `flatten`.
    /// Sequential: single init, direct append loop.
    pub(crate) fn flat_map_init_collect<T, B, R>(
        self,
        slice: &[T],
        init: impl Fn() -> B + Send + Sync,
        f: impl Fn(&mut B, &T, &mut Vec<R>) + Send + Sync,
    ) -> Vec<R>
    where
        T: Sync,
        B: Send,
        R: Send,
    {
        match self {
            Self::Sequential => {
                let mut bufs = init();
                let mut result = Vec::new();
                for item in slice {
                    f(&mut bufs, item, &mut result);
                }
                result
            }
            #[cfg(feature = "parallel")]
            Self::Parallel => {
                use rayon::prelude::*;
                slice
                    .par_iter()
                    .map_init(
                        || (init(), Vec::new()),
                        |(bufs, local), item| {
                            local.clear();
                            f(bufs, item, local);
                            std::mem::take(local)
                        },
                    )
                    .flatten()
                    .collect()
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::Par;

    #[test]
    fn chunks_mut_for_each_sequential() {
        let mut buf = vec![0u8; 10];
        Par::Sequential.chunks_mut_for_each(&mut buf, 3, |i, chunk| {
            for v in chunk.iter_mut() {
                *v = i as u8;
            }
        });
        assert_eq!(buf, vec![0, 0, 0, 1, 1, 1, 2, 2, 2, 3]);
    }

    #[test]
    fn map_init_collect_sequential() {
        let mut items = vec![1, 2, 3, 4, 5];
        let result = Par::Sequential.map_init_collect(
            &mut items,
            || 0usize, // running count
            |count, item| {
                *count += 1;
                if *item % 2 == 0 {
                    Some(*item * 10)
                } else {
                    None
                }
            },
        );
        assert_eq!(result, vec![20, 40]);
    }

    #[test]
    fn for_each_init_sequential() {
        let mut items = vec![1, 2, 3];
        Par::Sequential.for_each_init(
            &mut items,
            || 10i32,
            |base, item| {
                *item += *base;
            },
        );
        assert_eq!(items, vec![11, 12, 13]);
    }

    #[test]
    fn flat_map_init_collect_sequential() {
        let items = vec![1, 2, 3];
        let result = Par::Sequential.flat_map_init_collect(
            &items,
            || (),
            |_, &item, out| {
                for i in 0..item {
                    out.push(i);
                }
            },
        );
        // 1 -> [0], 2 -> [0,1], 3 -> [0,1,2]
        assert_eq!(result, vec![0, 0, 1, 0, 1, 2]);
    }
}
