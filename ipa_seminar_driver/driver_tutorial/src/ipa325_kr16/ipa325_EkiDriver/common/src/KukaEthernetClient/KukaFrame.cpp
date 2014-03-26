#include <KukaEthernetClient/KukaFrame.h>
#include <cstdio>
#include <cstring>

KukaFrame::KukaFrame(void)
{
	memset(&a, 0, sizeof(a));
}

KukaFrame::KukaFrame(float a1, float a2, float a3, float a4, float a5, float a6)
{
	a[0] = a1;
	a[1] = a2;
	a[2] = a3;
	a[3] = a4;
	a[4] = a5;
	a[5] = a6;
}

KukaFrame::KukaFrame(float* frame, int size)
{
    for(int i = 0; i < 6; i++)
    {
        if (i < size)
        {
            a[i] = frame[i];
        }
        else
        {
            a[i] = 0;
        }
    }
}

KukaFrame::KukaFrame(std::vector<float> frame)
{
    int size = frame.size();

        for(int i = 0; i < 6; i++)
        {
            if (i < size)
            {
                a[i] = frame[i];
            }
            else
            {
                a[i] = 0;
            }
        }
}

KukaFrame::~KukaFrame(void)
{
}

std::string KukaFrame::float2String(float value)
{
	char buffer[50];
	int n;
	n = sprintf(buffer, "%f", value);
	return std::string(buffer, n);
}

std::string KukaFrame::toString()
{
	std::string result = "";
	result += "{ X=";
	result += float2String(a[0]);
	result += " Y=";
	result += float2String(a[1]);
	result += " Z=";
	result += float2String(a[2]);
	result += " A=";
	result += float2String(a[3]);
	result += " B=";
	result += float2String(a[4]);
	result += " C=";
	result += float2String(a[5]);
	result += " }";
	return result;
}

