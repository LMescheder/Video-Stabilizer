template <class Traits>
class CameraStabilizer {
    public:
        CameraStabilizer() {
        }

        vector<Traits.Image> stabilize(vector<Traits.Image> input, Traits.Image refim) {
            auto refregions = Traits.computeRegions(refim);
            for (auto it = input.begin(); it <= input.end(); it++) {
                auto regions = Traits.computeRegions(*it);
                auto match = Traits.matchRegions(regions, refregions)
            }


        }

};
