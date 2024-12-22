
//	shader code for rendering MSTS models
//
/*
Copyright © 2024 Doug Jones

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/
#include <stdio.h>
#include <stdlib.h>

using namespace std;
#include <map>
#include <osg/Group>
#include <osg/Geometry>
#include <osg/MatrixTransform>
#include <osgViewer/Viewer>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osg/Texture2D>
#include <osgGA/TrackballManipulator>
#include <osg/NodeVisitor>
#include <osgShadow/ShadowedScene>
#include <osgShadow/ShadowMap>
#include <osg/PolygonMode>
#include <osg/Program>
#include <osg/Shader>
#include <osg/Uniform>

static const char* vertCode= R"(
varying vec3 N,L,V;
varying float occlusion;
void main()
{
	vec3 eye= vec3(gl_ModelViewMatrix * gl_Vertex);
	vec3 light= vec3(gl_ModelViewMatrix * gl_LightSource[0].position);
	V= -normalize(eye);
	N= normalize(vec3(gl_ModelViewMatrix * vec4(gl_Normal,0)));
	L= normalize(eye-light);
	gl_Position= gl_ModelViewProjectionMatrix * gl_Vertex;
	gl_TexCoord[0]= gl_MultiTexCoord0;
	occlusion= 1-gl_Color.x;
}
)";

static const char* phongFragCode= R"(
varying vec3 N,L,V;
varying float occlusion;
uniform sampler2D sampler0;
void main()
{
	vec3 nV= normalize(V);
	vec3 nN= normalize(N);
	vec3 nL= normalize(L);
	vec3 H= normalize(nL+nV);
	float NdotL= max(dot(nN,nL),0);
	float NdotH= max(dot(nN,H),0);
	vec4 baseColor= texture2D(sampler0,gl_TexCoord[0].st);
	vec3 c= (.6*(1-occlusion)+.4*NdotL)*baseColor.rgb;
	float specExp= 128;
	if (specExp > 0)
		c+= vec3(.4*NdotL*pow(max(0,NdotH),specExp));
	gl_FragColor= vec4(c,baseColor.a);
}
)";

static const char* pbrFragCode= R"(
varying vec3 N,L,V;
varying float occlusion;
uniform sampler2D sampler0;
uniform float roughness;
void main()
{
	vec3 nV= normalize(V);
	vec3 nN= normalize(N);
	vec3 nL= normalize(L);
	vec3 H= normalize(nL+nV);
	float NdotL= max(dot(nN,nL),0);
	float NdotH= max(dot(nN,H),0);
	float NdotV= max(dot(nN,nV),0);
	float VdotH= max(dot(nV,H),0);
	float alphaSq= roughness*roughness*roughness*roughness;
	float d= NdotH*NdotH*(alphaSq-1)+1;
	float D= alphaSq/d*d;
	float brdf= D/(NdotL+sqrt(alphaSq+(1-alphaSq)*NdotL*NdotL))/
	  (NdotV+sqrt(alphaSq+(1-alphaSq)*NdotV*NdotV));
	vec4 baseColor= texture2D(sampler0,gl_TexCoord[0].st);
	vec3 linearColor= pow(baseColor.rgb,vec3(2.2));
	float f0= .04;
	float F= f0 + (1-f0)*pow(1-VdotH,5);
	vec3 c= .6*(1-occlusion)*linearColor*gl_LightSource[0].ambient.xyz;
	c+= .4*NdotL*((1-F)*linearColor + vec3(F*brdf))*
	  gl_LightSource[0].diffuse.xyz;
	gl_FragColor= vec4(pow(c,vec3(1/2.2)),baseColor.a);
}
)";

void addShaders(osg::StateSet* stateSet, float roughness)
{
	if (roughness < 0)
		return;
	static osg::Program* prog= NULL;
	if (prog == NULL) {
		osg::Program* prog= new osg::Program;
		prog->addShader(new osg::Shader(osg::Shader::VERTEX,
		  vertCode));
		prog->addShader(new osg::Shader(
		  osg::Shader::FRAGMENT,pbrFragCode));
	}
	stateSet->setAttributeAndModes(prog);
	stateSet->addUniform(new osg::Uniform("roughness",roughness));
}
