"""Matching engine using KISS-Matcher (coarse) + small_gicp (fine)."""

import numpy as np
import time
from dataclasses import dataclass, field

import small_gicp


@dataclass
class CoarseParams:
    voxel_size: float = 0.3


@dataclass
class FineParams:
    method: str = "VGICP"
    max_iterations: int = 50
    max_correspondence_distance: float = 1.0
    voxel_resolution: float = 1.0
    num_threads: int = 4
    num_neighbors: int = 20


@dataclass
class QualityParams:
    min_fitness_score: float = 0.3
    max_rmse: float = 0.5


@dataclass
class MatchingResult:
    transformation: np.ndarray = field(default_factory=lambda: np.eye(4))
    fitness_score: float = 0.0
    rmse: float = 0.0
    success: bool = False
    message: str = ""
    elapsed_ms: float = 0.0


class MatchingEngine:
    def __init__(self, coarse_params: CoarseParams,
                 fine_params: FineParams,
                 quality_params: QualityParams,
                 logger):
        self.coarse_params = coarse_params
        self.fine_params = fine_params
        self.quality_params = quality_params
        self.logger = logger

    def match(self, reference_points: np.ndarray,
              source_points: np.ndarray) -> MatchingResult:
        start = time.time()
        result = MatchingResult()

        try:
            # --- Coarse alignment with KISS-Matcher ---
            coarse_T = self._coarse_match(reference_points, source_points)

            fitness_c, rmse_c = self._compute_metrics(reference_points, source_points, coarse_T)
            self.logger.info(f"Coarse: fitness={fitness_c:.3f}, rmse={rmse_c:.4f}")
            self.logger.info(f"Coarse translation: {coarse_T[:3, 3]}")

            # --- Fine alignment with small_gicp ---
            result = self._fine_match(reference_points, source_points, coarse_T)
            self.logger.info(f"Fine translation: {result.transformation[:3, 3]}")

        except Exception as e:
            import traceback; traceback.print_exc()
            result.success = False
            result.message = f"Matching failed: {str(e)}"

        result.elapsed_ms = (time.time() - start) * 1000.0

        # Quality check
        if result.fitness_score < self.quality_params.min_fitness_score:
            result.success = False
            result.message = (f"Fitness {result.fitness_score:.3f} below threshold "
                              f"{self.quality_params.min_fitness_score}")
        elif result.rmse > self.quality_params.max_rmse:
            result.success = False
            result.message = (f"RMSE {result.rmse:.4f} above threshold "
                              f"{self.quality_params.max_rmse}")
        else:
            result.success = True
            result.message = "Matching succeeded"
        return result

    def match_with_initial_guess(self, reference_points: np.ndarray,
                                  source_points: np.ndarray,
                                  initial_guess: np.ndarray) -> MatchingResult:
        """Fine matching only, using a provided initial guess."""
        start = time.time()
        result = self._fine_match(reference_points, source_points, initial_guess)
        result.elapsed_ms = (time.time() - start) * 1000.0
        return result

    def _coarse_match(self, reference_points, source_points):
        """KISS-Matcher global matching."""
        from kiss_matcher import KISSMatcher, KISSMatcherConfig

        config = KISSMatcherConfig()
        config.voxel_size = self.coarse_params.voxel_size
        matcher = KISSMatcher(config)

        src_matched_list, tgt_matched_list = matcher.match(source_points, reference_points)

        self.logger.info(f"KISS-Matcher correspondences: {len(src_matched_list)}")

        if len(src_matched_list) < 3:
            raise RuntimeError(f"Too few correspondences ({len(src_matched_list)})")

        src_matched = np.array(src_matched_list).T  # (3, N)
        tgt_matched = np.array(tgt_matched_list).T  # (3, N)

        solution = matcher.solve(src_matched, tgt_matched)

        T = np.eye(4)
        T[:3, :3] = solution.rotation
        T[:3, 3] = solution.translation

        if not self._is_valid(T):
            raise RuntimeError("KISS-Matcher returned invalid transformation")

        return T

    def _is_valid(self, T):
        """Check if transformation matrix is valid."""
        if T is None:
            return False
        try:
            T = np.asarray(T, dtype=np.float64)
        except (ValueError, TypeError):
            return False
        if T.shape != (4, 4):
            return False
        if np.any(np.isnan(T)) or np.any(np.isinf(T)):
            return False
        det = np.linalg.det(T[:3, :3])
        if abs(det - 1.0) > 0.01:
            return False
        return True

    def _fine_match(self, reference_points: np.ndarray,
                    source_points: np.ndarray,
                    initial_guess: np.ndarray) -> MatchingResult:
        """small_gicp local matching."""
        result = MatchingResult()

        reg_result = small_gicp.align(
            target_points=reference_points.astype(np.float64),
            source_points=source_points.astype(np.float64),
            init_T_target_source=initial_guess.astype(np.float64),
            registration_type=self.fine_params.method,
            voxel_resolution=self.fine_params.voxel_resolution,
            max_correspondence_distance=self.fine_params.max_correspondence_distance,
            num_threads=self.fine_params.num_threads,
            # num_neighbors=self.fine_params.num_neighbors,
            max_iterations=self.fine_params.max_iterations,
        )

        result.transformation = reg_result.T_target_source
        result.success = reg_result.converged
        result.message = "Converged" if reg_result.converged else "Did not converge"

        fitness, rmse = self._compute_metrics(
            reference_points, source_points, result.transformation
        )
        result.fitness_score = fitness
        result.rmse = rmse

        return result

    def _compute_metrics(self, reference_points: np.ndarray,
                         source_points: np.ndarray,
                         transformation: np.ndarray) -> tuple:
        """Compute fitness score and RMSE after matching."""
        R = transformation[:3, :3]
        t = transformation[:3, 3]
        aligned = (R @ source_points.T).T + t

        import scipy.spatial
        tree = scipy.spatial.KDTree(reference_points)

        distances, _ = tree.query(aligned)
        inlier_threshold = self.fine_params.max_correspondence_distance
        inlier_mask = distances < inlier_threshold

        fitness = np.sum(inlier_mask) / len(aligned)
        if np.sum(inlier_mask) > 0:
            rmse = np.sqrt(np.mean(distances[inlier_mask] ** 2))
        else:
            rmse = float('inf')

        return fitness, rmse