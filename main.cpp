#include "maths.h"

#include <iostream>

constexpr double EPS = 1e-7;

inline std::istream &operator>>(std::istream &is, Vector3D &v) { return is >> v.X >> v.Y >> v.Z; }

int main() {
    Vector3D a, b;
    std::cout << "Введите координаты 1-го отрезка (start X Y Z, end X Y Z):\n";
    if (!(std::cin >> a >> b)) {
        std::cerr << "Ошибка ввода первого отрезка!\n";
        return 1;
    }

    Vector3D c, d;
    std::cout << "Введите координаты 2-го отрезка (start X Y Z, end X Y Z):\n";
    if (!(std::cin >> c >> d)) {
        std::cerr << "Ошибка ввода второго отрезка!\n";
        return 1;
    }

    Segment3D x{a, b};
    Segment3D y{c, d};

    auto result = Intersect(x, y, EPS);

    if (auto *vec = std::get_if<Vector3D>(&result)) {
        std::cout << "Точка пересечения: " << vec->X << " " << vec->Y << " " << vec->Z << '\n';
    } else if (auto *seg = std::get_if<Segment3D>(&result)) {
        std::cout << "Отрезок пересечения: (" << seg->start.X << ", " << seg->start.Y << ", "
                  << seg->start.Z << ") - (" << seg->end.X << ", " << seg->end.Y << ", "
                  << seg->end.Z << ")\n";
    } else {
        std::cout << "Нет пересечения\n";
    }

    return 0;
}