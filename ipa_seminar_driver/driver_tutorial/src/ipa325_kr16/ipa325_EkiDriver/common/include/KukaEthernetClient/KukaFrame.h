#pragma once
#include <string>
#include <vector>
class KukaFrame
{
public:
	KukaFrame(void);
	KukaFrame(float a1, float a2, float a3, float a4, float a5, float a6);
        KukaFrame(float* frame, int size);
        KukaFrame(std::vector<float> frame);
	~KukaFrame(void);

	std::string toString();

private:
	std::string float2String(float value);

public:
	float a[6];

};

