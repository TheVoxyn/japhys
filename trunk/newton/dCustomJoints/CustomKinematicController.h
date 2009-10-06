

// CustomKinematicController.h: interface for the CustomKinematicController class.
//
//////////////////////////////////////////////////////////////////////

#ifndef __CUSTOM_KINEMATIC_CONTROLLER_H__
#define __CUSTOM_KINEMATIC_CONTROLLER_H__


#include "NewtonCustomJoint.h"

class JOINTLIBRARY_API CustomKinematicController: public NewtonCustomJoint
{
	public:
	CustomKinematicController (const NewtonBody* body, const dVector& attachmentPointInGlobalSpace);
	virtual ~CustomKinematicController();

	void SetPickMode (int mode);
	void SetMaxLinearFriction(dFloat accel); 
	void SetMaxAngularFriction(dFloat alpha); 
	
	void SetTargetRotation (const dQuaternion& rotation); 
	void SetTargetPosit (const dVector& posit); 
	void SetTargetMatrix (const dMatrix& matrix); 

	dMatrix GetTargetMatrix () const;

	protected:
	virtual void SubmitConstraints (dFloat timestep, int threadIndex);
	virtual void GetInfo (NewtonJointRecord* info) const;

	int m_pickMode;
	int m_autoSlepState;
	dFloat m_maxLinearFriction;
	dFloat m_maxAngularFriction;
	dVector m_localHandle;
	dVector m_targetPosit;
	dQuaternion m_targetRot;
};

#endif // !defined(AFX_CustomKinematicController_H__EAE1E36C_6FDF_4D86_B4EE_855E3D1046F4__INCLUDED_)
