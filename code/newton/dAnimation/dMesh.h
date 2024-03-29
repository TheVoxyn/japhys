/* Copyright (c) <2009> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/


#ifndef _D_MESH_H_
#define _D_MESH_H_

#include <dAnimationStdAfx.h>
#include <dList.h>
#include <dModel.h>
#include <dVector.h>
#include <dBaseHierarchy.h>
#include <dMathDefines.h>
#include <dRefCounter.h>

class TiXmlElement;
class dModel;


class dSubMesh
{
	public:
	dSubMesh ();
	~dSubMesh ();

	void AllocIndexData (int indexCount);
	int m_indexCount;
//	unsigned short *m_indexes;
	unsigned *m_indexes;
	unsigned m_textureHandle;

	dFloat m_shiness;
	dVector m_ambient;
	dVector m_diffuse;
	dVector m_specular;
	char m_textureName[64];
};


class dMesh: public dList<dSubMesh>, virtual public dRefCounter  
{
	public:
	enum dMeshType
	{
		D_STATIC_MESH,
		D_SKIN_MESH,
	};

	class dWeightList
	{
		public: 
		struct dBoneWeightIndex
		{
			int m_index[4];
		};

		dWeightList(int vertexCoiunt);
		~dWeightList();
		void SetBindingPose(dMesh* mesh, const dModel& model); 

		int m_bonesCount;
		const dBone* m_rootBone;
		const dBone** m_boneNodes;
		dVector* m_vertexWeight;
		dBoneWeightIndex* m_boneWeightIndex;
		dMatrix* m_bindingMatrices;
	};

	dMesh(dMeshType type);

	void AllocVertexData (int vertexCount);
	dMeshType GetType() const;

	void CalculateAABB (dVector& min, dVector& max) const;

	static void Load(const char* fileName, dList<dMesh*>& list, dLoaderContext& context);
	static void Save(const char* fileName, const dList<dMesh*>& list);

	TiXmlElement* ConvertToXMLNode () const;
	dSubMesh* AddSubMesh();

	protected:
	~dMesh();

	public:
	dMeshType m_type;
	int m_boneID;
	int m_vertexCount;
	dFloat *m_uv;
	dFloat *m_vertex;
	dFloat *m_normal;
	dWeightList* m_weighList;

	char m_name[D_NAME_STRING_LENGTH];
};




#endif 

