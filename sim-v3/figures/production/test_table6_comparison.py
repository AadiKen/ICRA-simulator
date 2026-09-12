from __future__ import annotations

import copy
import unittest

from figures.production.table6_comparison import validate_publication


VALID = {
    "artifact_kind": "publication-competitor-comparison",
    "rows": [{
        "tool": "example",
        "setup": {"value": "measured setup", "methodology": "measured"},
        "determinism": {"value": "not tested", "methodology": "not-obtainable"},
    }],
}


class Table6ValidationTest(unittest.TestCase):
    def test_accepts_normalized_claims(self) -> None:
        self.assertEqual(validate_publication(VALID), VALID["rows"])

    def test_rejects_missing_methodology(self) -> None:
        data = copy.deepcopy(VALID)
        del data["rows"][0]["determinism"]["methodology"]
        with self.assertRaisesRegex(ValueError, "invalid methodology tag"):
            validate_publication(data)

    def test_rejects_non_normalized_methodology(self) -> None:
        data = copy.deepcopy(VALID)
        data["rows"][0]["setup"]["methodology"] = "mixed"
        with self.assertRaisesRegex(ValueError, "invalid methodology tag"):
            validate_publication(data)


if __name__ == "__main__":
    unittest.main()
