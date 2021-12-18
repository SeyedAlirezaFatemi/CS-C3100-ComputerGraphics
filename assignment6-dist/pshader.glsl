#version 330

// control
uniform bool hasNormals;
uniform bool useTextures;
uniform bool useNormalMap;
uniform bool setDiffuseToZero;
uniform bool setSpecularToZero;

// 0: with lighting, 1: diffuse texture only,
// 2: normal map texture only, 3: final normal only
// 4: GGX distribution term D, 5: GGX geometry term G
// 6: Fresnel term Fr
uniform int renderMode;

// material parameters
uniform vec4 diffuseUniform;
uniform vec3 specularUniform;
uniform mat3 normalToCamera;
uniform mat4 posToCamera;
uniform float roughness;

// how deep the normal mapped bumps are
uniform float normalMapScale;

// texture samplers
uniform sampler2D diffuseSampler;
uniform sampler2D specularSampler;
uniform sampler2D normalSampler;

// lighting information, in camera space
uniform int  numLights;
uniform vec3 lightIntensities[8];
uniform vec3 lightDirections[8];

// interpolated inputs from vertex shader
in vec3 positionVarying;
in vec3 normalVarying;
in vec4 colorVarying;
in vec3 tangentVarying;
in vec2 texCoordVarying;

// output color
out vec4 outColor;

// inputs for shadow mapping
in vec2 shadowUV[3]; // location of the current fragment in the lights space
in float lightDist[3];
uniform sampler2D shadowSampler[3];
uniform bool shadowMaps;
uniform float shadowEps;

// this doesn't change
const float PI = 3.14159265359;

// The GGX distribution function D
// YOUR CODE HERE (R3)
float D(vec3 N, vec3 H) {
	return 1.;
}

// The Smith geometry term G
// YOUR CODE HERE (R4)
float G1(vec3 X, vec3 H) {
	return 1.;
}
float G(vec3 V, vec3 L, vec3 H) {
	return 1.;
}

// Fr is the Fresnel equation for dielectrics
// YOUR CODE HERE (R5)
float Fr(vec3 L, vec3 H) {
	const float n1 = 1.0; // air
	const float n2 = 1.4; // surface

	return 1.0;
}

// 4: GGX distribution term D, 5: GGX geometry term G
// 6: Fresnel term Fr, 7: GGX normalization term

// Cook-Torrance BRDF:
float CookTorrance(vec3 N, vec3 H, vec3 V, vec3 L) {
	switch (renderMode) {
	// debug modes..
	case 4: return (dot(N, L)>=.0)?D(N, H):.0;
	case 5: return G(V, L, N)*.1;
	case 6: return (dot(N, L)>=.0)?Fr(L, H):.0;
	default:
	// actual Cook-Torrance
	return Fr(L, H) * D(N, H) * G(V, L, N) / (4 * abs(dot(N, V) * dot(N, L)));
	}
}

void main()
{
	vec4 diffuseColor = diffuseUniform * colorVarying;
	vec4 specularColor = vec4(1.);

	if (useTextures)
	{
		// YOUR CODE HERE (R1)
		// Fetch the diffuse material albedos at the texture coordinates of the fragment.
		// This should be a one-liner and the same for both diffuse and specular.
		// diffuseColor = ...
		// specularColor = ...
	}

	// diffuse only?
	if (renderMode == 1)
	{
		outColor = vec4(diffuseColor.rgb, 1);
		return;
	}

	vec3 mappedNormal = normalize(normalToCamera * normalVarying);

	if (useNormalMap)
	{
		// YOUR CODE HERE (R1)
		// Fetch the object space normal from the normal map and scale it.
		// Then transform to camera space and assign to mappedNormal.
		// Don't forget to normalize!
		vec3 normalFromTexture = vec3(.0);
		// normalFromTexture = ...
			
		// debug display: normals as read from the texture
		if (renderMode == 2)
		{
			outColor = vec4(normalFromTexture*0.5 + 0.5, 1);
			return;
		}
	}

	// debug display: camera space normals
	if (renderMode == 3)
	{
		outColor = vec4(mappedNormal*0.5 + 0.5, 1);
		return;
	}

	vec3 N = mappedNormal;

	// YOUR CODE HERE (R3)
	// Compute the to-viewer vector V which you'll need in the loop
	// below for the specular computation.
	vec3 V = vec3(.0);

	// add the contribution of all lights to the answer
	vec3 answer = vec3(.0);

	for (int i = 0; i < numLights; ++i)
	{
		vec3 light_contribution = vec3(.0);

		// YOUR CODE HERE (R2)
		// Compute the diffuse shading contribution of this light.

		vec3 L = normalize(lightDirections[i]);
		vec3 Li = lightIntensities[i];
		vec3 diffuse;

		// YOUR CODE HERE (R3, R4, R5)
		// Compute the GGX specular contribution of this light.
		vec3 specular;

		if (setDiffuseToZero)
			diffuse = vec3(0, 0, 0);

		if (setSpecularToZero)
			specular = vec3(0, 0, 0);

		if (shadowMaps) {
			// YOUR SHADOWS HERE: use lightDist and shadowUV, maybe modify Li
			// this point is in a shadow is some point is closer to the light than this
			// (try also adding a small value to either of those and see what happens)
			//float shadow = 1.0f; // placeholder
		}

		if (renderMode >= 4) // debug mode; just sum up the specular distribution for each light
			answer += vec3(specular);
		else
			answer += light_contribution;
	}
	outColor = vec4(answer, 1);
}
