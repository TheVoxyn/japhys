#if !defined(AFX_CUSTOM_PLAYER_CONTROLLER_INCLUDED)
#define AFX_CUSTOM_PLAYER_CONTROLLER_INCLUDED


#include "NewtonCustomJoint.h"

class JOINTLIBRARY_API CustomPlayerController: public NewtonCustomJoint
{
	public:
	enum PlayerState
	{
		m_onLand,
		m_onFreeFall,
		m_onIlligalRamp,
	};

	CustomPlayerController (const dMatrix& localFrame, const NewtonBody* child, dFloat maxStairStepFactor, dFloat cushion);
	virtual ~CustomPlayerController();

	dFloat GetMaxSlope () const;
	void SetMaxSlope (dFloat maxSlopeAngleIndRadian);
	void SetVelocity (dFloat forwardSpeed, dFloat sideSpeed, dFloat heading);
	dMatrix CalculateVisualMatrix () const;
	
/*
	virtual bool CanPushBody (const NewtonBody* hitBody) const {return true;}
	const NewtonCollision* GetDynamicsSensorShape () const;
*/	
	const NewtonCollision* GetSensorShape () const;


	protected:
	virtual void SubmitConstraints (dFloat timestep, int threadIndex);

	private:
	void PlayerOnLand (dFloat timestep, int threadIndex);
	void PlayerOnRamp (dFloat timestep, int threadIndex);
	void PlayerOnFreeFall (dFloat timestep, int threadIndex);
	void KinematicMotion (dFloat timestep, int threadIndex);

	dVector CalculateVelocity (const dVector& velocSrc, dFloat timestep, const dVector& upDir, dFloat elevation, int threadIndex) const;

	

	static void KinematicMotion (const NewtonJoint* userJoint, dFloat timestep, int threadIndex);
	static unsigned ConvexStaticCastPrefilter(const NewtonBody* body, const NewtonCollision* collision, void* userData);
	static unsigned ConvexAllBodyCastPrefilter(const NewtonBody* body, const NewtonCollision* collision, void* userData);
//	static dFloat FindFloorCallback(const NewtonBody* body, const dFloat* hitNormal, int collisionID, void* userData, dFloat intersetParam);

	protected:
	struct CastFilterData
	{	
		CastFilterData (const NewtonBody* me)
		{
			m_count = 1;
			m_filter[0] = me;
		}
		int m_count;
		const NewtonBody* m_filter[8];
	};
/*
	struct FindFloorData
	{
		FindFloorData (const NewtonBody* me)
			:m_normal (0.0f, 1.0f, 0.0f, 0.0f)
		{
			m_me = me;
			m_param = 2.0f;
			m_hitBody = NULL;
		}

		dVector m_normal;
		dFloat m_param;
		const NewtonBody* m_me; 
		const NewtonBody* m_hitBody; 
	};
*/
/*

	virtual void GetInfo (NewtonJointRecord* info) const;
	static unsigned ConvexDynamicCastPrefilter(const NewtonBody* body, const NewtonCollision* collision, void* userData);
	NewtonCollision* m_dynamicsCollisionShape;
*/
	

	
	dFloat m_heading;
	dFloat m_loweCap;
	dFloat m_maxSlope;
	dFloat m_maxRadius;
	dFloat m_sideSpeed;
	dFloat m_restitution;
	dFloat m_forwardSpeed;
	dFloat m_playerHeight;
	dFloat m_stairHeight;
	dFloat m_kinematicCushion;
	dVector m_gravity;
	dMatrix m_localMatrix0;
	dMatrix m_localMatrix1;
	PlayerState m_playerState;
	NewtonCollision* m_bodySensorShape;
	NewtonCollision* m_stairSensorShape;
	NewtonCollision* m_bodyFloorSensorShape;
};

#endif