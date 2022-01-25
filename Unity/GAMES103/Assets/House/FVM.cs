using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.IO;

public class FVM : MonoBehaviour
{
	float dt 			= 0.002f;
    float mass 			= 1;
	float stiffness_0	= 7500.0f;
    float stiffness_1 	= 5000.0f;
    float damp			= 0.999f;

	int[] Tet;
	int tet_number;			//The number of tetrahedra

	Vector3[] Force;
	Vector3[] V;
	Vector3[] X;
	int number;				//The number of vertices

	Matrix4x4[] inv_Dm;

	//For Laplacian smoothing.
	Vector3[] V_sum;
	int[] V_num;

    Vector3 gravity     = new Vector3(0.0f, -9.8f, 0.0f);
    float restitution   = 0.5f;                            // for collision

    bool useSVD = false;

    SVD svd = new SVD();

    // Start is called before the first frame update
    void Start()
    {
    	// FILO IO: Read the house model from files.
    	// The model is from Jonathan Schewchuk's Stellar lib.
    	{
    		string fileContent = File.ReadAllText("Assets/House/house2.ele");
    		string[] Strings = fileContent.Split(new char[]{' ', '\t', '\r', '\n'}, StringSplitOptions.RemoveEmptyEntries);
    		
    		tet_number = int.Parse(Strings[0]);
        	Tet = new int[tet_number*4];

    		for (int tet = 0; tet < tet_number; tet++)
    		{
				Tet[tet * 4 + 0] = int.Parse(Strings[tet * 5 + 4]) - 1;
				Tet[tet * 4 + 1] = int.Parse(Strings[tet * 5 + 5]) - 1;
				Tet[tet * 4 + 2] = int.Parse(Strings[tet * 5 + 6]) - 1;
				Tet[tet * 4 + 3] = int.Parse(Strings[tet * 5 + 7]) - 1;
			}
    	}
    	{
			string fileContent = File.ReadAllText("Assets/House/house2.node");
    		string[] Strings = fileContent.Split(new char[]{' ', '\t', '\r', '\n'}, StringSplitOptions.RemoveEmptyEntries);
    		number = int.Parse(Strings[0]);
    		X = new Vector3[number];
       		for (int i = 0; i < number; i++)
       		{
       			X[i].x = float.Parse(Strings[i * 5 + 5]) * 0.4f;
       			X[i].y = float.Parse(Strings[i * 5 + 6]) * 0.4f;
       			X[i].z = float.Parse(Strings[i * 5 + 7]) * 0.4f;
       		}
    		//Centralize the model.
	    	Vector3 center = Vector3.zero;
	    	for (int i = 0; i < number; i++)
			{
				center += X[i];
			}
	    	center = center / number;
	    	for (int i = 0; i < number; i++)
	    	{
	    		X[i] -= center;
	    		float temp = X[i].y;
	    		X[i].y = X[i].z;
	    		X[i].z = temp;
	    	}
		}
        /*tet_number=1;
        Tet = new int[tet_number*4];
        Tet[0]=0;
        Tet[1]=1;
        Tet[2]=2;
        Tet[3]=3;

        number=4;
        X = new Vector3[number];
        V = new Vector3[number];
        Force = new Vector3[number];
        X[0]= new Vector3(0, 0, 0);
        X[1]= new Vector3(1, 0, 0);
        X[2]= new Vector3(0, 1, 0);
        X[3]= new Vector3(0, 0, 1);*/


        //Create triangle mesh.
       	Vector3[] vertices = new Vector3[tet_number * 12];
        int vertex_number = 0;
        for(int tet = 0; tet < tet_number; tet++)
        {
        	vertices[vertex_number++] = X[Tet[tet * 4 + 0]];
        	vertices[vertex_number++] = X[Tet[tet * 4 + 2]];
        	vertices[vertex_number++] = X[Tet[tet * 4 + 1]];
      
        	vertices[vertex_number++] = X[Tet[tet * 4 + 0]];
        	vertices[vertex_number++] = X[Tet[tet * 4 + 3]];
        	vertices[vertex_number++] = X[Tet[tet * 4 + 2]];
      
        	vertices[vertex_number++] = X[Tet[tet * 4 + 0]];
        	vertices[vertex_number++] = X[Tet[tet * 4 + 1]];
        	vertices[vertex_number++] = X[Tet[tet * 4 + 3]];
      
        	vertices[vertex_number++] = X[Tet[tet * 4 + 1]];
        	vertices[vertex_number++] = X[Tet[tet * 4 + 2]];
        	vertices[vertex_number++] = X[Tet[tet * 4 + 3]];
        }

        int[] triangles = new int[tet_number * 12];
        for (int t = 0; t < tet_number * 4; t++)
        {
        	triangles[t * 3 + 0] = t * 3 + 0;
        	triangles[t * 3 + 1] = t * 3 + 1;
        	triangles[t * 3 + 2] = t * 3 + 2;
        }
        Mesh mesh = GetComponent<MeshFilter> ().mesh;
		mesh.vertices  = vertices;
		mesh.triangles = triangles;
		mesh.RecalculateNormals ();

		V 	  = new Vector3[number];
        Force = new Vector3[number];
        V_sum = new Vector3[number];
        V_num = new int[number];
        for (int i = 0; i < number; i++)
        {
            V[i] = Vector3.zero;
            Force[i] = Vector3.zero;
            V_sum[i] = Vector3.zero;
            V_num[i] = 0;
        }

        // allocate and assign inv_Dm
        inv_Dm = new Matrix4x4[tet_number];
        for (int t = 0; t < tet_number; t++)
        {
            inv_Dm[t] = Build_Edge_Matrix(t).inverse;
        }
    }

    Matrix4x4 Build_Edge_Matrix(int tet)
    {
        // build edge matrix here.
        Vector3 v0 = X[Tet[tet * 4 + 0]];
        Vector3 v1 = X[Tet[tet * 4 + 1]];
        Vector3 v2 = X[Tet[tet * 4 + 2]];
        Vector3 v3 = X[Tet[tet * 4 + 3]];

        Vector3 v10 = v1 - v0;
        Vector3 v20 = v2 - v0;
        Vector3 v30 = v3 - v0;

        Vector4 column0 = new Vector4(v10.x, v10.y, v10.z, 0.0f);
        Vector4 column1 = new Vector4(v20.x, v20.y, v20.z, 0.0f);
        Vector4 column2 = new Vector4(v30.x, v30.y, v30.z, 0.0f);
        Vector4 column3 = new Vector4(0.0f, 0.0f, 0.0f, 1.0f);

        Matrix4x4 ret = new Matrix4x4(column0, column1, column2, column3);

        return ret;
    }
    void Collision_Impulse(ref Vector3 particlePosition, ref Vector3 particleVelocity, Vector3 P, Vector3 N)
    {
        Vector3 offset = particlePosition - P;
        float dotValue = Vector3.Dot(offset, N);
        if (dotValue < 0)
        {
            if (Vector3.Dot(particleVelocity, N) < 0)
            {
                Vector3 vni = Vector3.Dot(particleVelocity, N) * N;
                Vector3 vti = particleVelocity - vni;

                //
                // Friction
                // Decrease muN to reduce oscillation
                //
                float muT = restitution;
                float muNScale = Mathf.Min(vni.magnitude, 1.0f);
                if (muNScale < 0.16f)
                {
                    muNScale = 0.0f;
                }
                float muN = restitution * muNScale;
                float a = Mathf.Max(1.0f - muT * (1.0f + muN) * vni.magnitude / vti.magnitude, 0.0f);

                Vector3 vniNew = -muN * vni;
                Vector3 vtiNew = a * vti;
                particleVelocity = vniNew + vtiNew;
            }

            particlePosition -= dotValue * N;
        }
    }

    void _Update()
    {
    	// Jump up.
		if (Input.GetKeyDown(KeyCode.Space))
    	{
    		for (int i = 0; i < number; i++)
            {
                V[i].y += 0.2f;
            }
    	}

    	for (int i = 0; i < number; i++)
    	{
            // Add gravity to Force.
            Force[i] = gravity;
    	}

        //
        // It seems that accessing Matrix4x4 members explicitly is faster than using [row, col] within for loop
        //
    	for (int tet = 0; tet < tet_number; tet++)
    	{
            // Deformation Gradient
            Matrix4x4 EdgeMatrix = Build_Edge_Matrix(tet);
            Matrix4x4 F = EdgeMatrix * inv_Dm[tet];

            // PK Stress
            Matrix4x4 P = Matrix4x4.identity;

            if (useSVD)
            {
                Matrix4x4 U = Matrix4x4.identity;
                Matrix4x4 S = Matrix4x4.identity;
                Matrix4x4 V = Matrix4x4.identity;

                svd.svd(F, ref U, ref S, ref V);
                float sigma0 = S.m00;
                float sigma1 = S.m11;
                float sigma2 = S.m22;

                float I = sigma0 * sigma0 + sigma1 * sigma1 + sigma2 * sigma2;
                float dEdI = stiffness_0 * (I - 3.0f) * 0.25f - stiffness_1 * 0.5f;
                float dEdII = stiffness_1 * 0.25f;
                float P0 = 2 * dEdI * sigma0 + 4 * dEdII * sigma0 * sigma0 * sigma0;
                float P1 = 2 * dEdI * sigma1 + 4 * dEdII * sigma1 * sigma1 * sigma1;
                float P2 = 2 * dEdI * sigma2 + 4 * dEdII * sigma2 * sigma2 * sigma2;

                Matrix4x4 diag = Matrix4x4.identity;
                diag.m00 = P0;
                diag.m11 = P1;
                diag.m22 = P2;

                P = U * diag * V.transpose;
            }
            else
            {
                // Green Strain
                // G = (F^T * F - I) / 2
                Matrix4x4 G = F.transpose * F;
                G.m00 = (G.m00 - 1.0f) * 0.5f;
                G.m01 *= 0.5f;
                G.m02 *= 0.5f;

                G.m10 *= 0.5f;
                G.m11 = (G.m11 - 1.0f) * 0.5f;
                G.m12 *= 0.5f;

                G.m20 *= 0.5f;
                G.m22 = (G.m22 - 1.0f) * 0.5f;
                G.m22 *= 0.5f;

                // Second PK Stress
                // S = 2 * s1 * G + s0 * tr(G) * I,
                Matrix4x4 S = Matrix4x4.zero;
                float trace = G[0, 0] + G[1, 1] + G[2, 2];

                float coffa = 2.0f * stiffness_1;
                float coffb = stiffness_0 * trace;
                S.m00 = coffa * G.m00 + coffb;
                S.m01 = coffa * G.m01;
                S.m02 = coffa * G.m02;

                S.m10 = coffa * G.m10;
                S.m11 = coffa * G.m11 + coffb;
                S.m12 = coffa * G.m12;

                S.m20 = coffa * G.m20;
                S.m21 = coffa * G.m21;
                S.m22 = coffa * G.m22 + coffb;
                P = F * S;
            }

            // Elastic Force
            // Force matrix = P * Dm^(-T) / (-6 * det(Dm^-1))
            Matrix4x4 ForceMat = P * inv_Dm[tet].transpose;
            float det = inv_Dm[tet].determinant;
            float coff = -1.0f / (6.0f * det);
            ForceMat.m00 *= coff;
            ForceMat.m01 *= coff;
            ForceMat.m02 *= coff;

            ForceMat.m10 *= coff;
            ForceMat.m11 *= coff;
            ForceMat.m12 *= coff;

            ForceMat.m20 *= coff;
            ForceMat.m21 *= coff;
            ForceMat.m22 *= coff;

            Vector3 force1 = new Vector3(ForceMat.m00, ForceMat.m10, ForceMat.m20);
            Vector3 force2 = new Vector3(ForceMat.m01, ForceMat.m11, ForceMat.m21);
            Vector3 force3 = new Vector3(ForceMat.m02, ForceMat.m12, ForceMat.m22);
            Vector3 force0 = (force1 + force2 + force3) * -1.0f;

            Force[Tet[tet * 4 + 0]] += force0;
            Force[Tet[tet * 4 + 1]] += force1;
            Force[Tet[tet * 4 + 2]] += force2;
            Force[Tet[tet * 4 + 3]] += force3;
        }

        for (int i = 0; i < number; i++)
    	{
            // Update V here.
            V[i] += Force[i] / mass * dt;
            V[i] *= damp;

            // Prepare smoothing
            V_sum[i] = Vector3.zero;
            V_num[i] = 0;
        }

        // Laplacian smoothing
        for (int tet = 0; tet < tet_number; tet++)
        {
            for (int i = 0; i < 4; i++)
            {
                int index = Tet[tet * 4 + i];
                V_sum[index] += V[index];
                V_num[index] += 1;
                for (int j = 0; j < 4; j++)
                {
                    if (i != j)
                    {
                        int otherIndex = Tet[tet * 4 + j];
                        V_sum[index] += V[otherIndex];
                        V_num[index] += 1;
                    }
                }
            }
        }

        for (int i = 0; i < number; i++)
        {
            // Average velocity as smoothing
            if (V_num[i] > 1)
            {
                V[i] = V_sum[i] / V_num[i];
            }

            // Update X here.
            X[i] += V[i] * dt;

            // (Particle) collision with floor.
            Collision_Impulse(ref X[i], ref V[i], new Vector3(0, -2.99f, 0), new Vector3(0, 1, 0));
        }
    }

    // Update is called once per frame
    void Update()
    {
    	for (int l = 0; l < 10; l++)
        {
            _Update();
        }

    	// Dump the vertex array for rendering.
    	Vector3[] vertices = new Vector3[tet_number * 12];
        int vertex_number = 0;
        for (int tet = 0; tet < tet_number; tet++)
        {
        	vertices[vertex_number++] = X[Tet[tet * 4 + 0]];
        	vertices[vertex_number++] = X[Tet[tet * 4 + 2]];
        	vertices[vertex_number++] = X[Tet[tet * 4 + 1]];
        	vertices[vertex_number++] = X[Tet[tet * 4 + 0]];
        	vertices[vertex_number++] = X[Tet[tet * 4 + 3]];
        	vertices[vertex_number++] = X[Tet[tet * 4 + 2]];
        	vertices[vertex_number++] = X[Tet[tet * 4 + 0]];
        	vertices[vertex_number++] = X[Tet[tet * 4 + 1]];
        	vertices[vertex_number++] = X[Tet[tet * 4 + 3]];
        	vertices[vertex_number++] = X[Tet[tet * 4 + 1]];
        	vertices[vertex_number++] = X[Tet[tet * 4 + 2]];
        	vertices[vertex_number++] = X[Tet[tet * 4 + 3]];
        }
        Mesh mesh = GetComponent<MeshFilter>().mesh;
		mesh.vertices = vertices;
		mesh.RecalculateNormals();
    }
}
