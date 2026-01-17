//--------------------------------------------------------------------------------------------------
/**
	@file		ragnarokplugin.h

	@author		Dirk Gregorius
	@version	0.1
	@date		12/30/2025

	Copyright(C) D. Gregorius. All rights reserved.
*/
//--------------------------------------------------------------------------------------------------
#pragma once

#include <veritas/veritas.h>

// Ragnarok
#include <ragnarok/ragnarok.h>

// Forward 
class VsRagnarokSphereShape;
class VsRagnarokCapsuleShape;
class VsRagnarokHull;
class VsRagnarokHullShape;
class VsRagnarokMesh;
class VsRagnarokMeshShape;
class VsRagnarokBody;
class VsRagnarokWorld;
class VsRagnarokPlugin;


//--------------------------------------------------------------------------------------------------
// VsRagnarokHull
//--------------------------------------------------------------------------------------------------
class VsRagnarokHull : public IVsHull
	{
	public:
		// Construction / Destruction
		explicit VsRagnarokHull( RkHull* Hull );
		virtual ~VsRagnarokHull() override;

		// IVsHull
		virtual int GetVertexCount() const override;
		virtual const VsVector3* GetVertexPositions() const override;
		virtual const VsVector3* GetVertexNormals() const override;

		virtual int GetEdgeCount() const override;
		virtual const VsVector3* GetEdgePositions() const override;

		// Native Box3d type
		RkHull* GetNative() const;

	private:
		std::vector< VsVector3 > mVertexPositions;
		std::vector< VsVector3 > mVertexNormals;
		std::vector< VsVector3 > mEdgePositions;
		RkHull* mNative = nullptr;
	};


//--------------------------------------------------------------------------------------------------
// VsRagnarokHullShape
//--------------------------------------------------------------------------------------------------
class VsRagnarokHullShape : public IVsHullShape
	{
	public:
		// Construction / Destruction
		explicit VsRagnarokHullShape( VsRagnarokBody* Body, const VsRagnarokHull* Hull );
		virtual ~VsRagnarokHullShape();

		// IVsShape
		virtual VsShapeType GetType() const override;
		virtual IVsBody* GetBody() const override;
		virtual VsColor GetColor() const override;
		virtual void SetColor( const VsColor& Color ) override;

		// IVsHullShape
		virtual const IVsHull* GetHull() const override;

		// Material
		void SetFriction( float Friction );
		void SetRestitution( float Restitution );

		// Ragnarok
		RkHullShape* GetNative() const;

	private:
		VsRagnarokBody* mBody = nullptr;
		VsColor mColor = VS_COLOR_TRANSPARENT;
		const VsRagnarokHull* mHull = nullptr;
		RkHullShape* mNative = nullptr;
	};


//--------------------------------------------------------------------------------------------------
// VsRagnarokMesh
//--------------------------------------------------------------------------------------------------
class VsRagnarokMesh : public IVsMesh
	{
	public:
		// Construction / Destruction
		explicit VsRagnarokMesh();
		virtual ~VsRagnarokMesh() override;

		// IVsMesh
		virtual int GetVertexCount() const override;
		virtual const VsVector3* GetVertexPositions() const override;
		virtual const VsVector3* GetVertexNormals() const override;

	private:
		std::vector< VsVector3 > mVertexPositions;
		std::vector< VsVector3 > mVertexNormals;
	};


//--------------------------------------------------------------------------------------------------
// VsRagnarokBody
//--------------------------------------------------------------------------------------------------
class VsRagnarokBody : public IVsBody
	{
	public:
		// Construction / Destruction
		explicit VsRagnarokBody( VsRagnarokWorld* World, VsBodyType Type );
		virtual ~VsRagnarokBody() override;

		// World
		virtual IVsWorld* GetWorld() const override;

		// Type
		virtual VsBodyType GetType() const override;

		// Transform
		virtual VsVector3 GetPosition() const override;
		virtual void SetPosition( const VsVector3& Position ) override;
		virtual VsQuaternion GetOrientation() const override;
		virtual void SetOrientation( const VsQuaternion& Orientation ) override;

		// Velocity
		virtual VsVector3 GetLinearVelocity() const override;
		virtual void SetLinearVelocity( const VsVector3& LinearVelocity ) override;
		virtual VsVector3 GetAngularVelocity() const override;
		virtual void SetAngularVelocity( const VsVector3& AngularVelocity ) override;

		virtual void SetVelocityFromKeyframe( const VsFrame& Keyframe, float Timestep ) override;

		// Materials
		virtual float GetFriction() const override;
		virtual void SetFriction( float Friction ) override;
		virtual float GetRestitution() const override;
		virtual void SetRestitution( float Restitution ) override;

		// Shapes
		virtual IVsSphereShape* CreateSphere( const VsVector3& Center, float Radius ) override;
		virtual IVsCapsuleShape* CreateCapulse( const VsVector3& Center1, const VsVector3& Center2, float Radius ) override;
		virtual IVsHullShape* CreateHull( const IVsHull* Hull ) override;
		virtual IVsMeshShape* CreateMesh( const IVsMesh* Mesh ) override;
		virtual void DestroyShape( IVsShape* Shape ) override;

		virtual int GetShapeCount() const override;
		virtual IVsShape* GetShape( int ShapeIndex ) override;
		virtual const IVsShape* GetShape( int ShapeIndex ) const override;

		// Ragnarok
		RkBody* GetNative() const;

	private:
		VsRagnarokWorld* mWorld = nullptr;
		float mFriction = 0.6f;
		float mRestitution = 0.0f;
		std::vector< IVsShape* > mShapes;
		RkBody* mNative = nullptr;
	};


//--------------------------------------------------------------------------------------------------
// VsRagnarokWorld
//--------------------------------------------------------------------------------------------------
class VsRagnarokWorld : public IVsWorld
	{
	public:
		// Construction / Destruction
		explicit VsRagnarokWorld( VsRagnarokPlugin* Plugin );
		virtual ~VsRagnarokWorld() override;

		// Plugin
		virtual IVsPlugin* GetPlugin() const override;

		// Color
		virtual VsColor GetColor() const override;
		virtual void SetColor( const VsColor& Color ) override;

		// Events
		virtual void AddListener( IVsWorldListener* Listener ) override;
		virtual void RemoveListener( IVsWorldListener* Listener ) override;

		void NotifyBodyAdded( IVsBody* Body );
		void NotifyBodyRemoved( IVsBody* Body );
		void NotifyShapeAdded( IVsBody* Body, IVsShape* Shape );
		void NotifyShapeRemoved( IVsBody* Body, IVsShape* Shape );

		// Gravity
		virtual VsVector3 GetGravity() const override;
		virtual void SetGravity( const VsVector3& Gravity ) override;

		// Bodies
		virtual IVsBody* CreateBody( VsBodyType Type ) override;
		virtual void DestroyBody( IVsBody* Body ) override;
		virtual int GetBodyCount() const override;
		virtual IVsBody* GetBody( int BodyIndex ) override;
		virtual const IVsBody* GetBody( int BodyIndex ) const override;

		// Simulation
		virtual void Step( float Timestep ) override;

		// Ragnarok
		RkWorld* GetNative() const;

	private:
		VsRagnarokPlugin* mPlugin = nullptr;
		VsColor mColor = { 0.8f, 0.3f, 0.3f, 1.00f };
		std::vector< IVsWorldListener* > mListeners;
		std::vector< VsRagnarokBody* > mBodies;
		RkWorld* mNative = nullptr;
	};


//--------------------------------------------------------------------------------------------------
// VsRagnarokPlugin
//--------------------------------------------------------------------------------------------------
class VsRagnarokPlugin : public IVsPlugin
	{
	public:
		// Construction / Destruction
		explicit VsRagnarokPlugin( ImGuiContext* Context );
		virtual ~VsRagnarokPlugin() override;

		// Module
		virtual void Release() override;
		virtual const char* GetName() const override;
		virtual const char* GetVersion() const override;

		virtual bool IsEnabled() const override;
		virtual void SetEnabled( bool Enabled ) override;

		virtual bool OnInspectorGUI() override;

		// Hulls
		virtual IVsHull* CreateHull( int VertexCount, const VsVector3* Vertices ) override;
		virtual void DestroyHull( IVsHull* Hull ) override;

		virtual int GetHullCount() const override;
		virtual IVsHull* GetHull( int HullIndex ) override;
		virtual const IVsHull* GetHull( int HullIndex ) const override;

		// Meshes
		virtual IVsMesh* CreateMesh( int TriangleCount, const int* TriangleIndices, int VertexCount, const VsVector3* Vertices ) override;
		virtual void DestroyMesh( IVsMesh* Mesh ) override;

		virtual int GetMeshCount() const override;
		virtual IVsMesh* GetMesh( int MeshIndex ) override;
		virtual const IVsMesh* GetMesh( int MeshIndex ) const override;

		// Worlds
		virtual IVsWorld* CreateWorld() override;
		virtual void DestroyWorld( IVsWorld* World ) override;

		virtual int GetWorldCount() const override;
		virtual IVsWorld* GetWorld( int WorldIndex ) override;
		virtual const IVsWorld* GetWorld( int WorldIndex ) const override;

		// Ragnarok
		tf::Executor* GetExecutor() const;

	private:
		bool mEnabled = true;
		std::vector< VsRagnarokHull* > mHulls;
		std::vector< VsRagnarokMesh* > mMeshes;
		std::vector< VsRagnarokWorld* > mWorlds;

		tf::Executor* mExecutor = nullptr;
	};