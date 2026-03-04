const UNSET: u32 = 0xFFFF_FFFF;

/// Weighted union-find (disjoint-set) with path halving.
pub struct UnionFind {
    parent: Vec<u32>,
    size: Vec<u32>,
}

impl UnionFind {
    /// Create a new union-find with `n` elements, all initially unset.
    pub fn new(n: usize) -> Self {
        Self {
            parent: vec![UNSET; n],
            size: vec![0; n],
        }
    }

    /// Create an empty union-find with no elements and no allocation.
    pub fn empty() -> Self {
        Self {
            parent: Vec::new(),
            size: Vec::new(),
        }
    }

    /// Reset the union-find for `n` elements, reusing existing allocations.
    ///
    /// After reset, the state is identical to `UnionFind::new(n)`, but
    /// if the internal vectors already have sufficient capacity, no
    /// allocation occurs.
    pub fn reset(&mut self, n: usize) {
        self.parent.clear();
        self.parent.resize(n, UNSET);
        self.size.clear();
        self.size.resize(n, 0);
    }

    /// Find the representative of the set containing `id`, with path halving.
    ///
    /// If `id` has not been initialized, it becomes its own representative.
    pub fn find(&mut self, mut id: u32) -> u32 {
        if self.parent[id as usize] == UNSET {
            self.parent[id as usize] = id;
            return id;
        }
        while self.parent[id as usize] != id {
            let grandparent = self.parent[self.parent[id as usize] as usize];
            self.parent[id as usize] = grandparent;
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
        let sa = self.size[ra as usize] + 1;
        let sb = self.size[rb as usize] + 1;
        if sa > sb {
            self.parent[rb as usize] = ra;
            self.size[ra as usize] += sb;
            ra
        } else {
            self.parent[ra as usize] = rb;
            self.size[rb as usize] += sa;
            rb
        }
    }

    /// Get the size of the set containing `id` (including `id` itself).
    pub fn set_size(&mut self, id: u32) -> u32 {
        let r = self.find(id);
        self.size[r as usize] + 1
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
        assert_eq!(uf.parent.len(), 0);
        assert_eq!(uf.size.len(), 0);
    }

    #[test]
    fn reset_matches_new() {
        let fresh = UnionFind::new(10);
        let mut reused = UnionFind::empty();
        reused.reset(10);
        assert_eq!(fresh.parent, reused.parent);
        assert_eq!(fresh.size, reused.size);
    }

    #[test]
    fn reset_reuses_capacity() {
        let mut uf = UnionFind::new(100);
        // Use it
        uf.union(0, 1);
        uf.union(2, 3);
        // Reset to smaller size
        uf.reset(10);
        assert_eq!(uf.parent.len(), 10);
        assert!(uf.parent.capacity() >= 100);
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
