import hashlib
import json
import tempfile
import unittest
from pathlib import Path

from validation.figures import fig4_determinism


class FigurePipelineTest(unittest.TestCase):
    def test_figure4_is_byte_identical_and_has_provenance(self):
        first = fig4_determinism.build()
        digest = hashlib.sha256(first.read_bytes()).hexdigest()
        second = fig4_determinism.build()
        self.assertEqual(digest, hashlib.sha256(second.read_bytes()).hexdigest())
        provenance = json.loads(first.with_suffix(".provenance.json").read_text())
        self.assertEqual(provenance["figure"], "fig4_determinism")
        self.assertEqual(len(provenance["sources"]), 1)
        self.assertIn("git_sha", provenance)
        self.assertIn(b'<metadata id="provenance">', first.read_bytes())

    def test_wrong_cell_count_fails_loudly(self):
        report = json.loads(fig4_determinism.DEFAULT.read_text())
        report["cells"] = report["cells"][:-1]
        with tempfile.NamedTemporaryFile("w", suffix=".json", dir=fig4_determinism.ROOT, delete=False) as handle:
            json.dump(report, handle)
            path = Path(handle.name)
        try:
            with self.assertRaisesRegex(ValueError, "exactly 55 cells"):
                fig4_determinism.build([path])
        finally:
            path.unlink()


if __name__ == "__main__":
    unittest.main()
