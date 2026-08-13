export function clamp(value, min, max) {
    return Math.min(Math.max(value, min), max);
}

export function normalizeAngle(angle) {
    let normalized = angle;
    while (normalized > Math.PI) {
        normalized -= 2 * Math.PI;
    }
    while (normalized < -Math.PI) {
        normalized += 2 * Math.PI;
    }
    return normalized;
}

export function zeros(rows, cols) {
    return Array.from({length: rows}, () => Array(cols).fill(0));
}

export function matVecMul(matrix, vector) {
    return matrix.map((row) => row.reduce((sum, value, idx) => sum + value * vector[idx], 0));
}

export function addVec(a, b) {
    return a.map((value, idx) => value + b[idx]);
}

export function subVec(a, b) {
    return a.map((value, idx) => value - b[idx]);
}

export function scaleVec(v, scale) {
    return v.map((value) => value * scale);
}

export function addMatrices(a, b) {
    return a.map((row, r) => row.map((value, c) => value + b[r][c]));
}

export function diag(values) {
    const matrix = zeros(values.length, values.length);
    values.forEach((value, idx) => {
        matrix[idx][idx] = value;
    });
    return matrix;
}

export function invert3(m) {
    const a = m[0][0], b = m[0][1], c = m[0][2];
    const d = m[1][0], e = m[1][1], f = m[1][2];
    const g = m[2][0], h = m[2][1], i = m[2][2];
    const det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);

    if (Math.abs(det) < 1e-12) {
        throw new Error("Mass matrix is singular.");
    }

    const invDet = 1 / det;
    return [
        [(e * i - f * h) * invDet, (c * h - b * i) * invDet, (b * f - c * e) * invDet],
        [(f * g - d * i) * invDet, (a * i - c * g) * invDet, (c * d - a * f) * invDet],
        [(d * h - e * g) * invDet, (b * g - a * h) * invDet, (a * e - b * d) * invDet]
    ];
}

export function invertMatrix(matrix, tolerance = 1e-12) {
    const n = matrix.length;
    if (!n || matrix.some((row) => row.length !== n)) {
        throw new Error("Matrix must be non-empty and square.");
    }
    const augmented = matrix.map((row, r) => [
        ...row,
        ...Array.from({length: n}, (_, c) => r === c ? 1 : 0)
    ]);
    for (let col = 0; col < n; col += 1) {
        let pivot = col;
        for (let r = col + 1; r < n; r += 1) {
            if (Math.abs(augmented[r][col]) > Math.abs(augmented[pivot][col])) pivot = r;
        }
        if (!Number.isFinite(augmented[pivot][col]) || Math.abs(augmented[pivot][col]) < tolerance) {
            throw new Error("Mass matrix is singular or ill-conditioned.");
        }
        [augmented[col], augmented[pivot]] = [augmented[pivot], augmented[col]];
        const scale = augmented[col][col];
        augmented[col] = augmented[col].map((value) => value / scale);
        for (let r = 0; r < n; r += 1) {
            if (r === col) continue;
            const factor = augmented[r][col];
            augmented[r] = augmented[r].map((value, c) => value - factor * augmented[col][c]);
        }
    }
    return augmented.map((row) => row.slice(n));
}

export function isSymmetric(matrix, tolerance = 1e-10) {
    return matrix.every((row, r) => row.every((value, c) =>
        Math.abs(value - matrix[c][r]) <= tolerance
    ));
}

export function isPositiveDefinite(matrix, tolerance = 1e-12) {
    if (!isSymmetric(matrix)) return false;
    const n = matrix.length;
    const l = zeros(n, n);
    for (let i = 0; i < n; i += 1) {
        for (let j = 0; j <= i; j += 1) {
            let sum = matrix[i][j];
            for (let k = 0; k < j; k += 1) sum -= l[i][k] * l[j][k];
            if (i === j) {
                if (!Number.isFinite(sum) || sum <= tolerance) return false;
                l[i][j] = Math.sqrt(sum);
            }
            else {
                l[i][j] = sum / l[j][j];
            }
        }
    }
    return true;
}

export function assertSkewSymmetric(matrix, tolerance = 1e-9) {
    for (let r = 0; r < matrix.length; r += 1) {
        for (let c = 0; c < matrix.length; c += 1) {
            if (Math.abs(matrix[r][c] + matrix[c][r]) > tolerance) {
                return false;
            }
        }
    }
    return true;
}
