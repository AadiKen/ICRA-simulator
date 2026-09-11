from __future__ import annotations
import copy,json,unittest
from validation.figures.fig2_geography import DEFAULTS,SITES,validate_inputs

class GeographyValidationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        values=[json.loads(path.read_text()) for path in DEFAULTS]
        cls.current,cls.sf,cls.boston=values[:3]
        cls.masks={site:value for site,value in zip((row[0] for row in SITES),values[3:7])}
        cls.enc={row["site"]:row for row in values[7]["results"]}
    def test_shapes_are_valid(self): validate_inputs(self.current,self.sf,self.boston,self.masks,self.enc)
    def test_missing_mask_fails(self):
        masks=copy.deepcopy(self.masks);del masks["miami"]
        with self.assertRaisesRegex(ValueError,"four RTOFS masks"): validate_inputs(self.current,self.sf,self.boston,masks,self.enc)
    def test_incomplete_wind_fails(self):
        wind=copy.deepcopy(self.boston);wind["wind"]["matches"].pop()
        with self.assertRaisesRegex(ValueError,"72-hour wind"): validate_inputs(self.current,self.sf,wind,self.masks,self.enc)

if __name__=="__main__": unittest.main()
