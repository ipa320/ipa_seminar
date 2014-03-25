#pragma once
#include <string>
#include <vector>
class KukaAxis
{
public:
	KukaAxis(void);
	KukaAxis(float a1, float a2, float a3, float a4, float a5, float a6);
	KukaAxis(float* axes, int size);
	KukaAxis(std::vector<float> axes);
	~KukaAxis(void);

	std::string toString();
	//int fromString(std::string value);
        std::vector<double> toStdVector();

private:
	std::string float2String(float value);

public:
	float a[6];

};

