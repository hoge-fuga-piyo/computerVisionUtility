#ifndef MATHUTIL_HPP
#define MATHUTIL_HPP

class MathUtil {
public:
	static double convertDegreeToRadian(double degree);
	static double convertRadianToDegree(double radian);
	static double clamp(double v, double low, double high);
	static unsigned long long combination(unsigned int all_num, unsigned int select_num);

private:

};

#endif
