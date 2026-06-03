"""Visit-frequency map for D2OC: penalizes re-exploring already-visited cells.

Shares grid geometry (rows, cols) with DensityMap so indices are interchangeable.
"""
import numpy as np


class VisitMap:
    def __init__(self, rows, cols, kernel_radius=1, decay=0.0):
        self.rows = int(rows)
        self.cols = int(cols)
        self.kernel_radius = int(kernel_radius)
        self.decay = float(decay)
        self.counts = np.zeros((self.rows, self.cols), dtype=np.float64)
        self.total = 0.0
        self._kernel = self._build_kernel(self.kernel_radius)

    @staticmethod
    def _build_kernel(radius):
        ks = np.arange(-radius, radius + 1)
        kx, ky = np.meshgrid(ks, ks)
        kernel = np.exp(-0.5 * (kx ** 2 + ky ** 2) / ((radius + 0.1) ** 2))
        return kernel / (kernel.sum() + 1e-12)

    def register(self, col, row):
        """Add a Gaussian visit bump centered on (col, row); bump total by 1."""
        if self.decay > 0.0:
            self.counts *= self.decay

        r = self.kernel_radius
        r0, r1 = row - r, row + r + 1
        c0, c1 = col - r, col + r + 1

        kr0 = max(0, -r0)
        kc0 = max(0, -c0)
        dr0, dc0 = max(0, r0), max(0, c0)
        dr1, dc1 = min(self.rows, r1), min(self.cols, c1)
        if dr1 <= dr0 or dc1 <= dc0:
            self.total += 1.0
            return

        kr1 = kr0 + (dr1 - dr0)
        kc1 = kc0 + (dc1 - dc0)
        self.counts[dr0:dr1, dc0:dc1] += self._kernel[kr0:kr1, kc0:kc1]
        self.total += 1.0

    def frequency(self, rows_idx, cols_idx):
        """Visit frequency at the given (row, col) index arrays."""
        rows_idx = np.asarray(rows_idx, dtype=np.intp)
        cols_idx = np.asarray(cols_idx, dtype=np.intp)
        return self.counts[rows_idx, cols_idx].astype(np.float64)
