#include <KukaEthernetClient/KukaAxis.h>
//#include "Poco/StringTokenizer.h"
#include <cstdio>
#include <cstring>

//using Poco::StringTokenizer;
using namespace std;

KukaAxis::KukaAxis(void)
{
	memset(&a, 0, sizeof(a));
}

KukaAxis::KukaAxis(float a1, float a2, float a3, float a4, float a5, float a6)
{
	a[0] = a1;
	a[1] = a2;
	a[2] = a3;
	a[3] = a4;
	a[4] = a5;
	a[5] = a6;
}

KukaAxis::KukaAxis(float* axes, int size)
{
        for(int i = 0; i < 6; i++)
	{
            if (i < size)
            {
                a[i] = axes[i];
            }
            else
            {
                a[i] = 0;
            }
	}
}

KukaAxis::KukaAxis(std::vector<float> axes)
{
    int size = axes.size();

        for(int i = 0; i < 6; i++)
	{
            if (i < size)
            {
		a[i] = axes[i];
            }
            else
            {
                a[i] = 0;
            }
	}
}

KukaAxis::~KukaAxis(void)
{
}

std::string KukaAxis::float2String(float value)
{
	char buffer[50];
	int n;
	n = sprintf(buffer, "%f", value);
	return std::string(buffer, n);
}

std::string KukaAxis::toString()
{
	std::string result = "";
	result += "{ A1=";
	result += float2String(a[0]);
	result += ", A2=";
	result += float2String(a[1]);
	result += ", A3=";
	result += float2String(a[2]);
	result += ", A4=";
	result += float2String(a[3]);
	result += ", A5=";
	result += float2String(a[4]);
	result += ", A6=";
	result += float2String(a[5]);
	result += " }";
	return result;
}

std::vector<double> KukaAxis::toStdVector()
{
    std::vector<double> vec;
    for (int i=0; i<6; i++)
    {
        vec.push_back(a[i]);
    }
    return vec;
}

/*
int KukaAxis::fromString(std::string value)
{
	bool error[6] = {false}; 
	StringTokenizer ArgumentTokenizer(value, "{},", StringTokenizer::TOK_IGNORE_EMPTY);
	StringTokenizer::Iterator ArgumentIter = ArgumentTokenizer.begin();
	while(ArgumentIter != ArgumentTokenizer.end())
	{
		StringTokenizer ValueTokenizer(*ArgumentIter, "=", StringTokenizer::TOK_IGNORE_EMPTY);
		StringTokenizer::Iterator valueIter = ValueTokenizer.begin();
		if(valueIter != ValueTokenizer.end())
		{
			if (*valueIter == "A1")
			{
				a[0] = atof((*++valueIter).c_str());
				error[0] = true;
			}
			else if (*valueIter == "A2")
			{
				a[1] = atof((*++valueIter).c_str());
				error[1] = true;
			}
			else if (*valueIter == "A3")
			{
				a[2] = atof((*++valueIter).c_str());
				error[2] = true;
			}
			else if (*valueIter == "A4")
			{
				a[3] = atof((*++valueIter).c_str());
				error[3] = true;
			}
			else if (*valueIter == "A5")
			{
				a[4] = atof((*++valueIter).c_str());
				error[4] = true;
			}
			else if (*valueIter == "A6")
			{
				a[5] = atof((*++valueIter).c_str());
				error[5] = true;
			}
		}
		++ArgumentIter;
	}

	// Test
	bool result = true;
	for (int i = 0; i < 6; i++)
	{
		result = error[i] && result;
	}

	if (!result) // Test fehlgeschlagen
	{
		memset(&a, 0, sizeof(a));
		return 1;
	}
	return 0;
}*/
