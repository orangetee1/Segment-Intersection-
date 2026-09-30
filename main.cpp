#include "maths.h"

#include <iostream>

int main() {
    Vector3D a{0, 2, 1};
    Vector3D b{4, 6, 9};

    Vector3D c{3, 2, 7};
    Vector3D d{-1, 8, 3};

    Segment3D x{a, b};
    Segment3D y{c, d};

    auto result = Intersect(x, y);

    if (result.has_value()) {
        std::cout << result->X << " " << result->Y << " " << result->Z << std::endl;
    }

    return 0;
};