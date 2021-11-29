using UnityEngine;
using System.Collections;

public class Rigid_Bunny : MonoBehaviour 
{
	bool launched 		= false;
	float dt 			= 0.015f;
	Vector3 v 			= new Vector3(0, 0, 0);	// velocity
	Vector3 w 			= new Vector3(0, 0, 0);	// angular velocity
	
	float mass;									// mass
	Matrix4x4 I_ref;							// reference inertia

	float linear_decay	= 0.999f;				// for velocity decay
	float angular_decay	= 0.98f;				
	float restitution 	= 0.5f;					// for collision


	// Use this for initialization
	void Start() 
	{		
		Mesh mesh = GetComponent<MeshFilter>().mesh;
		Vector3[] vertices = mesh.vertices;

		float m=1;
		mass=0;
		for (int i=0; i<vertices.Length; i++) 
		{
			mass += m;
			float diag=m*vertices[i].sqrMagnitude;
			I_ref[0, 0]+=diag;
			I_ref[1, 1]+=diag;
			I_ref[2, 2]+=diag;
			I_ref[0, 0]-=m*vertices[i][0]*vertices[i][0];
			I_ref[0, 1]-=m*vertices[i][0]*vertices[i][1];
			I_ref[0, 2]-=m*vertices[i][0]*vertices[i][2];
			I_ref[1, 0]-=m*vertices[i][1]*vertices[i][0];
			I_ref[1, 1]-=m*vertices[i][1]*vertices[i][1];
			I_ref[1, 2]-=m*vertices[i][1]*vertices[i][2];
			I_ref[2, 0]-=m*vertices[i][2]*vertices[i][0];
			I_ref[2, 1]-=m*vertices[i][2]*vertices[i][1];
			I_ref[2, 2]-=m*vertices[i][2]*vertices[i][2];
		}
		I_ref [3, 3] = 1;
	}
	
	Matrix4x4 Get_Cross_Matrix(Vector3 a)
	{
		//Get the cross product matrix of vector a
		Matrix4x4 A = Matrix4x4.zero;
		A [0, 0] = 0; 
		A [0, 1] = -a [2]; 
		A [0, 2] = a [1]; 
		A [1, 0] = a [2]; 
		A [1, 1] = 0; 
		A [1, 2] = -a [0]; 
		A [2, 0] = -a [1]; 
		A [2, 1] = a [0]; 
		A [2, 2] = 0; 
		A [3, 3] = 1;
		return A;
	}

	// In this function, update v and w by the impulse due to the collision with
	//a plane <P, N>
	void Collision_Impulse(Vector3 P, Vector3 N)
	{
		Mesh mesh = GetComponent<MeshFilter>().mesh;
		Vector3[] vertices = mesh.vertices;
		Matrix4x4 rotationMatrix = Matrix4x4.Rotate(transform.rotation);

		int numCollidingVertices = 0;
		Vector3 sumCollidingPosition = Vector3.zero;
		Vector3 sumCollidingVelocity = Vector3.zero;
		foreach (Vector3 position in vertices)
		{
			Vector3 rotatedDirection = rotationMatrix.MultiplyPoint3x4(position);
			Vector3 vertexPosition = transform.position + rotatedDirection;
			if (Vector3.Dot(vertexPosition - P, N) < 0)
			{
				Vector3 vertexVelocity = v + Vector3.Cross(w, rotatedDirection);
				if (Vector3.Dot(vertexVelocity, N) < 0)
				{
					sumCollidingPosition += vertexPosition;
					sumCollidingVelocity += vertexVelocity;
					numCollidingVertices++;
				}
			}
		}

		if (numCollidingVertices > 0)
		{
			Vector3 Rri = sumCollidingPosition / numCollidingVertices - transform.position;
			Vector3 vi = sumCollidingVelocity / numCollidingVertices;

			Vector3 vni = Vector3.Dot(vi, N) * N;
			Vector3 vti = vi - vni;

			float mut = 0.5f;
			float mun = 0.5f;
			float a = Mathf.Max(1.0f - mut * (1.0f + mun) * vni.magnitude / vti.magnitude, 0.0f);

			Vector3 vniNew = -mun * vni;
			Vector3 vtiNew = a * vti;
			Vector3 viNew = vniNew + vtiNew;

			float inversedMass = 1.0f / mass;
			Matrix4x4 K = new Matrix4x4(new Vector4(inversedMass, 0, 0, 0), new Vector4(0, inversedMass, 0, 0), new Vector4(0, 0, inversedMass, 0), new Vector4(0, 0, 0, inversedMass));
			Matrix4x4 crossMatrix = Get_Cross_Matrix(Rri);
			Matrix4x4 Temp = crossMatrix * I_ref.inverse * crossMatrix;
			K.SetColumn(0, K.GetColumn(0) - Temp.GetColumn(0));
			K.SetColumn(1, K.GetColumn(1) - Temp.GetColumn(1));
			K.SetColumn(2, K.GetColumn(2) - Temp.GetColumn(2));
			K.SetColumn(3, K.GetColumn(3) - Temp.GetColumn(3));

			Vector3 j = K.inverse.MultiplyPoint3x4(viNew - vi);

			v = v + j * inversedMass;
			w = w + I_ref.inverse.MultiplyPoint3x4(Vector3.Cross(Rri, j));
		}
	}

	void Update_Linear_Velocity()
	{
		Vector3 gravity = new Vector3(0.0f, -9.8f, 0.0f);
		v += gravity * dt;
		v *= linear_decay;
	}

	void Update_Angular_Velocity()
	{
		w *= angular_decay;
	}

	void Update_Linear_Status(ref Vector3 position)
	{
		position += v * dt;
	}

	void Update_Angular_Status(ref Quaternion rotation)
	{
		Vector3 wt = w * 0.5f * dt;
		Quaternion deltaq = new Quaternion(wt.x, wt.y, wt.z, 0);
		deltaq *= rotation;
		rotation.x += deltaq.x;
		rotation.y += deltaq.y;
		rotation.z += deltaq.z;
		rotation.w += deltaq.w;
		rotation.Normalize();
	}

	// Update is called once per frame
	void Update() 
	{
		//Game Control
		if(Input.GetKey("r"))
		{
			transform.position = new Vector3 (0, 0.6f, 0);
			restitution = 0.5f;
			launched=false;
		}
		if(Input.GetKey("l"))
		{
			v = new Vector3 (5, 2, 0);
			launched=true;
		}

		// Part I: Update velocities
		if (launched)
		{
			Update_Linear_Velocity();
		}
		Update_Angular_Velocity();

		// Part II: Collision Impulse
		Collision_Impulse(new Vector3(0, 0.01f, 0), new Vector3(0, 1, 0));
		Collision_Impulse(new Vector3(2, 0, 0), new Vector3(-1, 0, 0));

		// Part III: Update position & orientation
		//Update linear status
		Vector3 x    = transform.position;
		//Update angular status
		Quaternion q = transform.rotation;

		if (launched)
		{
			Update_Linear_Status(ref x);
		}
		Update_Angular_Status(ref q);

		// Part IV: Assign to the object
		transform.position = x;
		transform.rotation = q;
	}
}
