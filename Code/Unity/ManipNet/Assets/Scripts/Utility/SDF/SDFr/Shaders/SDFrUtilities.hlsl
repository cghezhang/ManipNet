#ifndef XRA_RAYMARCH_UTILITIES
#define XRA_RAYMARCH_UTILITIES

float3 HsvToRgb(float3 c)
{
	const float4 K = float4(1.0, 2.0 / 3.0, 1.0 / 3.0, 3.0);
	float3 p = abs(frac(c.xxx + K.xyz) * 6.0 - K.www);
	return c.z * lerp(K.xxx, saturate(p - K.xxx), c.y);
}

float map(float value, float istart, float istop, float ostart, float ostop)
{
	float perc = (value - istart) / (istop - istart);
	value = perc * (ostop - ostart) + ostart;
	return value;
}

#endif