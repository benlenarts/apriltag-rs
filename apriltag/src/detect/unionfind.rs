const UNSET: u32 = 0xFFFF_FFFF;

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

/// Weighted union-find (disjoint-set) with path halving.
///
/// Parent and size are interleaved in a single `Vec<u64>` so that both
/// fields share a cache line, matching the C reference implementation's
/// layout and eliminating extra memory accesses.
pub struct UnionFind {
    /// Packed entries: low 32 bits = parent, high 32 bits = size.
    data: Vec<u64>,
}

impl UnionFind {
    /// Create a new union-find with `n` elements, all initially unset.
    pub fn new(n: usize) -> Self {
        Self {
            data: vec![pack(UNSET, 0); n],
        }
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
        self.data.clear();
        self.data.resize(n, pack(UNSET, 0));
    }

    /// Find the representative of the set containing `id`, with path halving.
    ///
    /// If `id` has not been initialized, it becomes its own representative.
    pub fn find(&mut self, mut id: u32) -> u32 {
        let parent = unpack_parent(self.data[id as usize]);
        if parent == UNSET {
            self.data[id as usize] = pack(id, 0);
            return id;
        }
        while unpack_parent(self.data[id as usize]) != id {
            let parent = unpack_parent(self.data[id as usize]);
            let grandparent = unpack_parent(self.data[parent as usize]);
            // Path halving: point to grandparent, preserving size
            let size = unpack_size(self.data[id as usize]);
            self.data[id as usize] = pack(grandparent, size);
            id = grandparent;
        }
        id
    }

    /// Union the sets containing `a` and `b`. Returns the new representative.
    ///
    /// Uses weighted union (larger tree becomes root).
    pub fn union(&mut self, a: u32, b: u32) -> u32 {
        let ra = self.find(a);
        let rb = self.find(b);
        if ra == rb {
            return ra;
        }
        let sa = unpack_size(self.data[ra as usize]) + 1;
        let sb = unpack_size(self.data[rb as usize]) + 1;
        if sa > sb {
            self.data[rb as usize] = pack(ra, unpack_size(self.data[rb as usize]));
            self.data[ra as usize] = pack(ra, unpack_size(self.data[ra as usize]) + sb);
            ra
        } else {
            self.data[ra as usize] = pack(rb, unpack_size(self.data[ra as usize]));
            self.data[rb as usize] = pack(rb, unpack_size(self.data[rb as usize]) + sa);
            rb
        }
    }

    /// Get the size of the set containing `id` (including `id` itself).
    pub fn set_size(&mut self, id: u32) -> u32 {
        let r = self.find(id);
        unpack_size(self.data[r as usize]) + 1
    }
}

#[cfg(test)]
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
    fn path_halving_works() {
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
        assert_eq!(uf.data.len(), 0);
    }

    #[test]
    fn reset_matches_new() {
        let fresh = UnionFind::new(10);
        let mut reused = UnionFind::empty();
        reused.reset(10);
        assert_eq!(fresh.data, reused.data);
    }

    #[test]
    fn reset_reuses_capacity() {
        let mut uf = UnionFind::new(100);
        // Use it
        uf.union(0, 1);
        uf.union(2, 3);
        // Reset to smaller size
        uf.reset(10);
        assert_eq!(uf.data.len(), 10);
        assert!(uf.data.capacity() >= 100);
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
    fn union_same_set_returns_rep() {
        let mut uf = UnionFind::new(3);
        uf.union(0, 1);
        let r = uf.find(0);
        assert_eq!(uf.union(0, 1), r);
    }
}
