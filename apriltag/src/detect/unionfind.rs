#[cfg(feature = "parallel")]
use core::sync::atomic::{AtomicU64, Ordering};

/// Packed parent (low 32 bits) + size (high 32 bits) in a single u64.
/// Sharing a cache line eliminates one memory access per find/union step.
#[inline(always)]
fn pack(parent: u32, size: u32) -> u64 {
    (size as u64) << 32 | parent as u64
}

#[inline(always)]
fn unpack_parent(v: u64) -> u32 {
    v as u32
}

#[inline(always)]
fn unpack_size(v: u64) -> u32 {
    (v >> 32) as u32
}

// Type alias: AtomicU64 when parallel, plain u64 otherwise.
#[cfg(feature = "parallel")]
type Cell = AtomicU64;
#[cfg(not(feature = "parallel"))]
type Cell = u64;

/// Read the raw u64 value from a Cell.
#[cfg(feature = "parallel")]
#[inline(always)]
fn cell_read(c: &Cell) -> u64 {
    c.load(Ordering::Relaxed)
}

#[cfg(not(feature = "parallel"))]
#[inline(always)]
fn cell_read(c: &Cell) -> u64 {
    *c
}

/// Get a mutable reference to the inner u64 (zero-cost for AtomicU64 via get_mut).
#[cfg(feature = "parallel")]
#[inline(always)]
fn cell_get_mut(c: &mut Cell) -> &mut u64 {
    c.get_mut()
}

#[cfg(not(feature = "parallel"))]
#[inline(always)]
fn cell_get_mut(c: &mut Cell) -> &mut u64 {
    c
}

/// Create a Cell from a u64 value.
#[cfg(feature = "parallel")]
#[inline(always)]
fn cell_new(v: u64) -> Cell {
    AtomicU64::new(v)
}

#[cfg(not(feature = "parallel"))]
#[inline(always)]
fn cell_new(v: u64) -> Cell {
    v
}

/// Weighted union-find (disjoint-set) with path splitting.
///
/// Parent and size are interleaved in a single `Vec<u64>` so that both
/// fields share a cache line, matching the C reference implementation's
/// layout and eliminating extra memory accesses.
///
/// Elements are eagerly initialized: each element starts as its own
/// representative with size 0, eliminating a branch from `find()`.
///
/// When the `parallel` feature is enabled, internal storage uses `AtomicU64`
/// to support lock-free concurrent `find_shared`/`union_shared`. Exclusive
/// (`&mut self`) methods use `get_mut()` for zero-overhead non-atomic access.
pub struct UnionFind {
    /// Packed entries: low 32 bits = parent, high 32 bits = size.
    data: Vec<Cell>,
}

/// Initialize `data` so that each element is its own representative.
fn init_data(data: &mut Vec<Cell>, n: usize) {
    data.clear();
    data.reserve(n.saturating_sub(data.capacity()));
    data.extend((0..n as u32).map(|i| cell_new(pack(i, 0))));
}

impl UnionFind {
    /// Create a new union-find with `n` elements, each its own representative.
    pub fn new(n: usize) -> Self {
        let mut data = Vec::new();
        init_data(&mut data, n);
        Self { data }
    }

    /// Create an empty union-find with no elements and no allocation.
    pub fn empty() -> Self {
        Self { data: Vec::new() }
    }

    /// Reset the union-find for `n` elements, reusing existing allocations.
    ///
    /// After reset, the state is identical to `UnionFind::new(n)`, but
    /// if the internal vectors already have sufficient capacity, no
    /// allocation occurs.
    pub fn reset(&mut self, n: usize) {
        init_data(&mut self.data, n);
    }

    /// Find the representative of the set containing `id`, with path splitting.
    ///
    /// Path splitting points every node on the find path to its grandparent,
    /// then advances to the old parent (not the grandparent). This visits
    /// every node on the path, compressing more aggressively per traversal
    /// than path halving.
    #[inline]
    pub fn find(&mut self, mut id: u32) -> u32 {
        assert!((id as usize) < self.data.len());
        loop {
            let entry = *cell_get_mut(&mut self.data[id as usize]);
            let parent = unpack_parent(entry);
            if parent == id {
                return id;
            }
            let grandparent = unpack_parent(*cell_get_mut(&mut self.data[parent as usize]));
            // Path splitting: point to grandparent, advance to old parent
            *cell_get_mut(&mut self.data[id as usize]) = pack(grandparent, unpack_size(entry));
            id = parent;
        }
    }

    /// Union the sets containing `a` and `b`. Returns the new representative.
    ///
    /// Uses weighted union (larger tree becomes root).
    #[inline]
    pub fn union(&mut self, a: u32, b: u32) -> u32 {
        let ra = self.find(a);
        let rb = self.find(b);
        if ra == rb {
            return ra;
        }
        let sa = unpack_size(*cell_get_mut(&mut self.data[ra as usize])) + 1;
        let sb = unpack_size(*cell_get_mut(&mut self.data[rb as usize])) + 1;
        if sa > sb {
            *cell_get_mut(&mut self.data[rb as usize]) =
                pack(ra, unpack_size(*cell_get_mut(&mut self.data[rb as usize])));
            *cell_get_mut(&mut self.data[ra as usize]) = pack(
                ra,
                unpack_size(*cell_get_mut(&mut self.data[ra as usize])) + sb,
            );
            ra
        } else {
            *cell_get_mut(&mut self.data[ra as usize]) =
                pack(rb, unpack_size(*cell_get_mut(&mut self.data[ra as usize])));
            *cell_get_mut(&mut self.data[rb as usize]) = pack(
                rb,
                unpack_size(*cell_get_mut(&mut self.data[rb as usize])) + sa,
            );
            rb
        }
    }

    /// Get the size of the set containing `id` (including `id` itself).
    pub fn set_size(&mut self, id: u32) -> u32 {
        let r = self.find(id);
        unpack_size(*cell_get_mut(&mut self.data[r as usize])) + 1
    }

    /// Get the size of a set given its root representative directly.
    ///
    /// The caller must pass a value previously returned by `find()`.
    /// This avoids a redundant `find()` call when the root is already known.
    #[inline(always)]
    pub fn root_size(&self, root: u32) -> u32 {
        unpack_size(cell_read(&self.data[root as usize])) + 1
    }

    /// Flatten all paths so every element points directly to its root.
    ///
    /// After this call, [`find_flat`](Self::find_flat) returns the correct
    /// representative in O(1) without mutation, enabling read-only concurrent
    /// access from multiple threads.
    pub fn flatten(&mut self) {
        for i in 0..self.data.len() as u32 {
            let root = self.find(i);
            let entry = *cell_get_mut(&mut self.data[i as usize]);
            *cell_get_mut(&mut self.data[i as usize]) = pack(root, unpack_size(entry));
        }
    }

    /// Find the representative of the set containing `id` in O(1).
    ///
    /// Requires [`flatten`](Self::flatten) to have been called first.
    /// Unlike [`find`](Self::find), this takes `&self` and can be called
    /// concurrently from multiple threads.
    #[inline(always)]
    pub fn find_flat(&self, id: u32) -> u32 {
        unpack_parent(cell_read(&self.data[id as usize]))
    }

    /// Find the representative of `id` using lock-free path splitting.
    ///
    /// Uses `Relaxed` atomic loads/stores. Stale reads may cause extra
    /// traversal but cannot produce incorrect final results — rayon's
    /// join/collect barrier ensures all writes are visible before the
    /// serial stitching phase reads them.
    #[cfg(feature = "parallel")]
    #[inline]
    pub fn find_shared(&self, mut id: u32) -> u32 {
        loop {
            let entry = self.data[id as usize].load(Ordering::Relaxed);
            let parent = unpack_parent(entry);
            if parent == id {
                return id;
            }
            let gp_entry = self.data[parent as usize].load(Ordering::Relaxed);
            let grandparent = unpack_parent(gp_entry);
            // Best-effort path splitting: benign race if another thread
            // also updates this node — either update is valid.
            self.data[id as usize].store(pack(grandparent, unpack_size(entry)), Ordering::Relaxed);
            id = parent;
        }
    }

    /// Union the sets containing `a` and `b` using lock-free CAS.
    ///
    /// Returns the new representative. Uses `Relaxed` ordering —
    /// correctness does not depend on observing the latest root, only on
    /// the CAS loop eventually succeeding for each union.
    #[cfg(feature = "parallel")]
    pub fn union_shared(&self, a: u32, b: u32) -> u32 {
        loop {
            let mut ra = self.find_shared(a);
            let mut rb = self.find_shared(b);
            if ra == rb {
                return ra;
            }
            // Canonical ordering: smaller id becomes child to reduce contention
            let sa = unpack_size(self.data[ra as usize].load(Ordering::Relaxed));
            let sb = unpack_size(self.data[rb as usize].load(Ordering::Relaxed));
            if sa < sb || (sa == sb && ra > rb) {
                core::mem::swap(&mut ra, &mut rb);
            }
            // ra is the winner (larger set). Make rb point to ra.
            let rb_entry = self.data[rb as usize].load(Ordering::Relaxed);
            if unpack_parent(rb_entry) != rb {
                // rb is no longer a root — retry
                continue;
            }
            // CAS: set rb's parent to ra
            if self.data[rb as usize]
                .compare_exchange_weak(
                    rb_entry,
                    pack(ra, unpack_size(rb_entry)),
                    Ordering::Relaxed,
                    Ordering::Relaxed,
                )
                .is_ok()
            {
                // Best-effort size update on winner (only affects heuristic)
                let ra_entry = self.data[ra as usize].load(Ordering::Relaxed);
                let new_size = unpack_size(ra_entry) + unpack_size(rb_entry) + 1;
                let _ = self.data[ra as usize].compare_exchange(
                    ra_entry,
                    pack(ra, new_size),
                    Ordering::Relaxed,
                    Ordering::Relaxed,
                );
                return ra;
            }
            // CAS failed — retry
        }
    }

    /// Compare data contents for testing (AtomicU64 doesn't impl PartialEq).
    #[cfg(test)]
    fn data_eq(&self, other: &Self) -> bool {
        if self.data.len() != other.data.len() {
            return false;
        }
        self.data
            .iter()
            .zip(other.data.iter())
            .all(|(a, b)| cell_read(a) == cell_read(b))
    }

    /// Number of elements.
    #[cfg(test)]
    fn len(&self) -> usize {
        self.data.len()
    }

    /// Raw capacity.
    #[cfg(test)]
    fn capacity(&self) -> usize {
        self.data.capacity()
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use super::*;

    #[test]
    fn find_initializes_self() {
        let mut uf = UnionFind::new(5);
        assert_eq!(uf.find(3), 3);
    }

    #[test]
    fn union_merges_sets() {
        let mut uf = UnionFind::new(5);
        uf.union(0, 1);
        assert_eq!(uf.find(0), uf.find(1));
    }

    #[test]
    fn union_weighted_larger_becomes_root() {
        let mut uf = UnionFind::new(10);
        // Build a larger set {0,1,2}
        uf.union(0, 1);
        uf.union(0, 2);
        let r_large = uf.find(0);
        // Single element {5}
        uf.find(5);
        let root = uf.union(0, 5);
        // The larger set's rep should be the root
        assert_eq!(root, r_large);
        assert_eq!(uf.find(5), r_large);
    }

    #[test]
    fn set_size_correct() {
        let mut uf = UnionFind::new(5);
        assert_eq!(uf.set_size(0), 1);
        uf.union(0, 1);
        assert_eq!(uf.set_size(0), 2);
        assert_eq!(uf.set_size(1), 2);
        uf.union(0, 2);
        assert_eq!(uf.set_size(0), 3);
    }

    #[test]
    fn path_compression_works() {
        let mut uf = UnionFind::new(10);
        // Create a chain: 0→1→2→3
        uf.union(0, 1);
        uf.union(1, 2);
        uf.union(2, 3);
        // After find, path should be compressed
        let root = uf.find(0);
        assert_eq!(uf.find(0), root);
        assert_eq!(uf.find(1), root);
        assert_eq!(uf.find(2), root);
        assert_eq!(uf.find(3), root);
    }

    #[test]
    fn disjoint_sets_stay_separate() {
        let mut uf = UnionFind::new(4);
        uf.union(0, 1);
        uf.union(2, 3);
        assert_ne!(uf.find(0), uf.find(2));
    }

    #[test]
    fn empty_creates_no_allocation() {
        let uf = UnionFind::empty();
        assert_eq!(uf.len(), 0);
    }

    #[test]
    fn reset_matches_new() {
        let fresh = UnionFind::new(10);
        let mut reused = UnionFind::empty();
        reused.reset(10);
        assert!(fresh.data_eq(&reused));
    }

    #[test]
    fn reset_reuses_capacity() {
        let mut uf = UnionFind::new(100);
        // Use it
        uf.union(0, 1);
        uf.union(2, 3);
        // Reset to smaller size
        uf.reset(10);
        assert_eq!(uf.len(), 10);
        assert!(uf.capacity() >= 100);
        // Should work identically to fresh
        uf.union(0, 1);
        assert_eq!(uf.find(0), uf.find(1));
    }

    #[test]
    fn reset_clears_previous_unions() {
        let mut uf = UnionFind::new(5);
        uf.union(0, 1);
        uf.union(2, 3);
        uf.reset(5);
        // After reset, all elements should be unset (disjoint)
        for i in 0..5u32 {
            assert_eq!(uf.find(i), i);
        }
    }

    #[test]
    fn root_size_matches_set_size() {
        let mut uf = UnionFind::new(10);
        uf.union(0, 1);
        uf.union(0, 2);
        let root = uf.find(0);
        assert_eq!(uf.root_size(root), 3);
        assert_eq!(uf.root_size(root), uf.set_size(0));
    }

    #[test]
    fn union_same_set_returns_rep() {
        let mut uf = UnionFind::new(3);
        uf.union(0, 1);
        let r = uf.find(0);
        assert_eq!(uf.union(0, 1), r);
    }

    #[test]
    fn flatten_enables_find_flat() {
        let mut uf = UnionFind::new(10);
        uf.union(0, 1);
        uf.union(1, 2);
        uf.union(2, 3);
        uf.union(5, 6);
        uf.union(6, 7);

        uf.flatten();

        // All elements in {0,1,2,3} should have the same root
        let root_a = uf.find_flat(0);
        assert_eq!(uf.find_flat(1), root_a);
        assert_eq!(uf.find_flat(2), root_a);
        assert_eq!(uf.find_flat(3), root_a);

        // All elements in {5,6,7} should have the same root
        let root_b = uf.find_flat(5);
        assert_eq!(uf.find_flat(6), root_b);
        assert_eq!(uf.find_flat(7), root_b);

        // Singletons should point to themselves
        assert_eq!(uf.find_flat(4), 4);
        assert_eq!(uf.find_flat(8), 8);

        // Different sets should have different roots
        assert_ne!(root_a, root_b);
    }

    #[test]
    fn flatten_preserves_sizes() {
        let mut uf = UnionFind::new(5);
        uf.union(0, 1);
        uf.union(0, 2);
        let root = uf.find(0);
        let size_before = uf.root_size(root);
        uf.flatten();
        assert_eq!(uf.root_size(root), size_before);
    }

    #[cfg(feature = "parallel")]
    mod shared {
        use super::*;

        #[test]
        fn find_shared_returns_self_for_fresh() {
            let uf = UnionFind::new(5);
            assert_eq!(uf.find_shared(0), 0);
            assert_eq!(uf.find_shared(4), 4);
        }

        #[test]
        fn union_shared_merges_and_find_shared_agrees() {
            let uf = UnionFind::new(5);
            uf.union_shared(0, 1);
            assert_eq!(uf.find_shared(0), uf.find_shared(1));
        }

        #[test]
        fn union_shared_weighted_larger_wins() {
            let uf = UnionFind::new(10);
            uf.union_shared(0, 1);
            uf.union_shared(0, 2);
            let r_large = uf.find_shared(0);
            let root = uf.union_shared(0, 5);
            assert_eq!(root, r_large);
            assert_eq!(uf.find_shared(5), r_large);
        }

        #[test]
        fn concurrent_union_shared_all_merges_apply() {
            use std::sync::Arc;
            let n = 100u32;
            let uf = Arc::new(UnionFind::new(n as usize));
            // Spawn threads that each union a range of elements into element 0
            let nthreads = 4u32;
            let mut handles = Vec::new();
            for t in 0..nthreads {
                let uf = Arc::clone(&uf);
                let start = t * (n / nthreads) + 1;
                let end = if t == nthreads - 1 {
                    n
                } else {
                    (t + 1) * (n / nthreads) + 1
                };
                handles.push(std::thread::spawn(move || {
                    for i in start..end {
                        uf.union_shared(0, i);
                    }
                }));
            }
            for h in handles {
                h.join().unwrap();
            }
            // After all threads join, every element should share a root with 0
            let root = uf.find_shared(0);
            for i in 1..n {
                assert_eq!(
                    uf.find_shared(i),
                    root,
                    "element {i} not merged with root {root}"
                );
            }
        }
    }
}
