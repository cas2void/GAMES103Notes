using UnityEngine;
using System.Collections;

public class wave_motion : MonoBehaviour
{
    int size = 100;
    float rate = 0.005f;
    float gamma = 0.002f;
    float damping = 0.996f;
    float[,] old_h;
    float[,] low_h;
    float[,] vh;
    float[,] b;

    bool[,] cg_mask;
    float[,] cg_p;
    float[,] cg_r;
    float[,] cg_Ap;

    Vector3 cube_v = Vector3.zero;
    Vector3 cube_w = Vector3.zero;

    GameObject cube;
    GameObject block;

    float dt = 0.004f;
    float mass;
    Matrix4x4 I_ref;               // reference inertia
    float linear_decay = 0.9999f;
    float angular_decay = 0.98f;

    Vector3 gravity = new Vector3(0.0f, -9.8f, 0.0f);
    float buoyancy_scale = 0.3f;

    // Use this for initialization
    void Start()
    {
        Mesh mesh = GetComponent<MeshFilter>().mesh;
        mesh.Clear();

        Vector3[] X = new Vector3[size * size];

        for (int i = 0; i < size; i++)
        {
            for (int j = 0; j < size; j++)
            {
                X[i * size + j].x = i * 0.1f - size * 0.05f;
                X[i * size + j].y = 0;
                X[i * size + j].z = j * 0.1f - size * 0.05f;
            }
        }

        int[] T = new int[(size - 1) * (size - 1) * 6];
        int index = 0;
        for (int i = 0; i < size - 1; i++)
        {
            for (int j = 0; j < size - 1; j++)
            {
                T[index * 6 + 0] = (i + 0) * size + (j + 0);
                T[index * 6 + 1] = (i + 0) * size + (j + 1);
                T[index * 6 + 2] = (i + 1) * size + (j + 1);
                T[index * 6 + 3] = (i + 0) * size + (j + 0);
                T[index * 6 + 4] = (i + 1) * size + (j + 1);
                T[index * 6 + 5] = (i + 1) * size + (j + 0);
                index++;
            }
        }
        mesh.vertices = X;
        mesh.triangles = T;
        mesh.RecalculateNormals();

        low_h = new float[size, size];
        old_h = new float[size, size];
        vh = new float[size, size];
        b = new float[size, size];

        cg_mask = new bool[size, size];
        cg_p = new float[size, size];
        cg_r = new float[size, size];
        cg_Ap = new float[size, size];

        for (int i = 0; i < size; i++)
        {
            for (int j = 0; j < size; j++)
            {
                low_h[i, j] = 99999;
                old_h[i, j] = 0;
                vh[i, j] = 0;
            }
        }

        cube = GameObject.Find("Cube");
        block = GameObject.Find("Block");

        Vector3[] original_vertices = cube.GetComponent<MeshFilter>().mesh.vertices;

        float m = 0.5f;
        mass = 0;
        for (int i = 0; i < original_vertices.Length; i++)
        {
            mass += m;
            float diag = m * original_vertices[i].sqrMagnitude;
            I_ref[0, 0] += diag;
            I_ref[1, 1] += diag;
            I_ref[2, 2] += diag;
            I_ref[0, 0] -= m * original_vertices[i][0] * original_vertices[i][0];
            I_ref[0, 1] -= m * original_vertices[i][0] * original_vertices[i][1];
            I_ref[0, 2] -= m * original_vertices[i][0] * original_vertices[i][2];
            I_ref[1, 0] -= m * original_vertices[i][1] * original_vertices[i][0];
            I_ref[1, 1] -= m * original_vertices[i][1] * original_vertices[i][1];
            I_ref[1, 2] -= m * original_vertices[i][1] * original_vertices[i][2];
            I_ref[2, 0] -= m * original_vertices[i][2] * original_vertices[i][0];
            I_ref[2, 1] -= m * original_vertices[i][2] * original_vertices[i][1];
            I_ref[2, 2] -= m * original_vertices[i][2] * original_vertices[i][2];
        }
        I_ref[3, 3] = 1;
    }

    void A_Times(bool[,] mask, float[,] x, float[,] Ax, int li, int ui, int lj, int uj)
    {
        for (int i = li; i <= ui; i++)
        {
            for (int j = lj; j <= uj; j++)
            {
                if (i >= 0 && j >= 0 && i < size && j < size && mask[i, j])
                {
                    Ax[i, j] = 0;
                    if (i != 0)
                    {
                        Ax[i, j] -= x[i - 1, j] - x[i, j];
                    }
                    if (i != size - 1)
                    {
                        Ax[i, j] -= x[i + 1, j] - x[i, j];
                    }
                    if (j != 0)
                    {
                        Ax[i, j] -= x[i, j - 1] - x[i, j];
                    }
                    if (j != size - 1)
                    {
                        Ax[i, j] -= x[i, j + 1] - x[i, j];
                    }
                }
            }
        }
    }

    float Dot(bool[,] mask, float[,] x, float[,] y, int li, int ui, int lj, int uj)
    {
        float ret = 0;
        for (int i = li; i <= ui; i++)
        {
            for (int j = lj; j <= uj; j++)
            {
                if (i >= 0 && j >= 0 && i < size && j < size && mask[i, j])
                {
                    ret += x[i, j] * y[i, j];
                }
            }
        }
        return ret;
    }

    void Conjugate_Gradient(bool[,] mask, float[,] b, float[,] x, int li, int ui, int lj, int uj)
    {
        //Solve the Laplacian problem by CG.
        A_Times(mask, x, cg_r, li, ui, lj, uj);

        for (int i = li; i <= ui; i++)
        {
            for (int j = lj; j <= uj; j++)
            {
                if (i >= 0 && j >= 0 && i < size && j < size && mask[i, j])
                {
                    cg_p[i, j] = cg_r[i, j] = b[i, j] - cg_r[i, j];
                }
            }
        }

        float rk_norm = Dot(mask, cg_r, cg_r, li, ui, lj, uj);

        for (int k = 0; k < 128; k++)
        {
            if (rk_norm < 1e-10f)
            {
                break;
            }
            A_Times(mask, cg_p, cg_Ap, li, ui, lj, uj);
            float alpha = rk_norm / Dot(mask, cg_p, cg_Ap, li, ui, lj, uj);

            for (int i = li; i <= ui; i++)
            {
                for (int j = lj; j <= uj; j++)
                {
                    if (i >= 0 && j >= 0 && i < size && j < size && mask[i, j])
                    {
                        x[i, j] += alpha * cg_p[i, j];
                        cg_r[i, j] -= alpha * cg_Ap[i, j];
                    }
                }
            }

            float _rk_norm = Dot(mask, cg_r, cg_r, li, ui, lj, uj);
            float beta = _rk_norm / rk_norm;
            rk_norm = _rk_norm;

            for (int i = li; i <= ui; i++)
            {
                for (int j = lj; j <= uj; j++)
                {
                    if (i >= 0 && j >= 0 && i < size && j < size && mask[i, j])
                    {
                        cg_p[i, j] = cg_r[i, j] + beta * cg_p[i, j];
                    }
                }
            }
        }

    }

    void Neumann_Boundry(int i, int j, out int li, out int ui, out int lj, out int uj)
    {
        li = i - 1;
        ui = i + 1;
        lj = j - 1;
        uj = j + 1;

        if (i == 0)
        {
            li = i;
        }
        else if (i == size - 1)
        {
            ui = i;
        }

        if (j == 0)
        {
            lj = j;
        }
        else if (j == size - 1)
        {
            uj = j;
        }
    }

    void Solve_Virtual_Height(GameObject gameObject, float[,] virtualHeight, float[,] new_h, Vector3[,] vertices)
    {
        BoxCollider collider = gameObject.GetComponent<BoxCollider>();
        // Renderer and Collider components' bounds are measured in world space 
        Bounds bounds = collider.bounds;
        float boundsCenterY = bounds.center.y;

        // Contact range
        int li = size - 1;
        int ui = 0;
        int lj = size - 1;
        int uj = 0;

        for (int i = 0; i < size; i++)
        {
            for (int j = 0; j < size; j++)
            {
                // Initailize as no contact
                cg_mask[i, j] = false;
                low_h[i, j] = new_h[i, j];

                Vector3 waterVertex = new Vector3(vertices[i, j].x, boundsCenterY, vertices[i, j].z);
                if (bounds.Contains(waterVertex))
                {
                    // Arbitrary water bottom, -10 is enough for this demo.
                    float bottom = -10.0f;
                    // Shoot ray up from bottom
                    Ray ray = new Ray(new Vector3(waterVertex.x, bottom, waterVertex.z), Vector3.up);
                    if (collider.Raycast(ray, out RaycastHit hitInfo, 100.0f))
                    {
                        // Update contact range
                        if (i < li)
                        {
                            li = i;
                        }
                        else if (i > ui)
                        {
                            ui = i;
                        }

                        if (j < lj)
                        {
                            lj = j;
                        }
                        else if (j > uj)
                        {
                            uj = j;
                        }

                        // If contact, set depth as desired height
                        float depth = bottom + hitInfo.distance;
                        if (depth < 0)
                        {
                            cg_mask[i, j] = true;
                            low_h[i, j] = depth;
                        }
                    }

                    b[i, j] = (new_h[i, j] - low_h[i, j]) / rate;
                }
            }
        }

        Conjugate_Gradient(cg_mask, b, virtualHeight, li, ui, lj, uj);
    }

    void Water_Block_Coupling(GameObject gameObject, float[,] virtualHeight, Vector3[,] vertices)
    {
        BoxCollider collider = gameObject.GetComponent<BoxCollider>();
        // Renderer and Collider components' bounds are measured in world space 
        Bounds bounds = collider.bounds;
        float boundsCenterY = bounds.center.y;

        // Find all the vertice collided with the water
        int numCollidingVertices = 0;
        Vector3 sumCollidingPosition = Vector3.zero;
        float sumVirtualHeight = 0.0f;

        for (int i = 0; i < size; i++)
        {
            for (int j = 0; j < size; j++)
            {
                Vector3 waterVertex = new Vector3(vertices[i, j].x, boundsCenterY, vertices[i, j].z);
                if (bounds.Contains(waterVertex))
                {
                    // Arbitrary water bottom, -10 is enough for this demo.
                    float bottom = -10.0f;
                    // Shoot ray up from bottom
                    Ray ray = new Ray(new Vector3(waterVertex.x, bottom, waterVertex.z), Vector3.up);
                    if (collider.Raycast(ray, out RaycastHit hitInfo, 100.0f))
                    {
                        if (virtualHeight[i, j] > 0)
                        {
                            numCollidingVertices++;
                            sumCollidingPosition += hitInfo.point;
                            sumVirtualHeight += virtualHeight[i, j];
                        }
                    }
                }
            }
        }

        // Buoyancy
        Vector3 buoyancyForce = new Vector3(0.0f, sumVirtualHeight * -gravity.y * buoyancy_scale, 0.0f);

        // Decay velocity
        cube_v += gravity * dt;
        cube_v *= linear_decay;

        cube_w *= angular_decay;

        // Update linear status
        cube_v += buoyancyForce / mass * dt;
        gameObject.transform.position += cube_v * dt;

        // Calculate the torque
        Vector3 tao = Vector3.zero;
        if (numCollidingVertices > 0)
        {
            //
            // Average of all collided vertices
            //
            Vector3 Rri = sumCollidingPosition / numCollidingVertices - gameObject.transform.position;
            tao = Vector3.Cross(Rri, buoyancyForce) / mass;
        }

        // Update angular status
        Matrix4x4 rotationMatrix = Matrix4x4.Rotate(gameObject.transform.rotation);
        Matrix4x4 tempMatrix = rotationMatrix * I_ref * rotationMatrix.transpose;
        cube_w += tempMatrix.inverse.MultiplyVector(tao);

        Quaternion rotation = gameObject.transform.rotation;
        Vector3 wt = cube_w * 0.5f * dt;
        Quaternion deltaq = new Quaternion(wt.x, wt.y, wt.z, 0);
        deltaq *= rotation;
        rotation.x += deltaq.x;
        rotation.y += deltaq.y;
        rotation.z += deltaq.z;
        rotation.w += deltaq.w;
        rotation.Normalize();
        gameObject.transform.rotation = rotation;
    }

    void Shallow_Wave(float[,] old_h, float[,] h, float[,] new_h, Vector3[,] vertices)
    {
        //Step 1:
        // Compute new_h based on the shallow wave model.
        for (int i = 0; i < size; i++)
        {
            for (int j = 0; j < size; j++)
            {
                // Neumann boundary conditions
                Neumann_Boundry(i, j, out int lowerI, out int upperI, out int lowerJ, out int upperJ);
                new_h[i, j] = h[i, j] + (h[i, j] - old_h[i, j]) * damping + (h[lowerI, j] + h[upperI, j] + h[i, lowerJ] + h[i, upperJ] - 4.0f * h[i, j]) * rate;
            }
        }

        //Step 2: Block->Water coupling
        // for block 1, calculate low_h.
        // then set up b and cg_mask for conjugate gradient.
        // Solve the Poisson equation to obtain vh (virtual height).
        Solve_Virtual_Height(block, vh, new_h, vertices);


        // for block 2, calculate low_h.
        // then set up b and cg_mask for conjugate gradient.
        // Solve the Poisson equation to obtain vh (virtual height).
        Solve_Virtual_Height(cube, vh, new_h, vertices);

        // Diminish vh.
        for (int i = 0; i < size; i++)
        {
            for (int j = 0; j < size; j++)
            {
                vh[i, j] *= gamma;
            }
        }

        // Update new_h by vh.
        for (int i = 0; i < size; i++)
        {
            for (int j = 0; j < size; j++)
            {
                // Neumann boundary conditions
                Neumann_Boundry(i, j, out int lowerI, out int upperI, out int lowerJ, out int upperJ);
                new_h[i, j] += (vh[lowerI, j] + vh[upperI, j] + vh[i, lowerJ] + vh[i, upperJ] - 4.0f * vh[i, j]) * rate;
            }
        }

        //Step 3
        // old_h <- h; h <- new_h;
        for (int i = 0; i < size; i++)
        {
            for (int j = 0; j < size; j++)
            {
                old_h[i, j] = h[i, j];
                h[i, j] = new_h[i, j];
            }
        }

        //Step 4: Water->Block coupling.
        // Attach rigid body dynamics with implicit integration.
        // Here we couple water with the cube only, the same mechanics could be added to the block.
        Water_Block_Coupling(cube, vh, vertices);
    }


    // Update is called once per frame
    void Update()
    {
        Mesh mesh = GetComponent<MeshFilter>().mesh;
        Vector3[] X = mesh.vertices;
        float[,] new_h = new float[size, size];
        float[,] h = new float[size, size];
        Vector3[,] vertices = new Vector3[size, size];

        // Load X.y into h.
        for (int i = 0; i < size; i++)
        {
            for (int j = 0; j < size; j++)
            {
                h[i, j] = X[i * size + j].y;
                vertices[i, j] = X[i * size + j];
            }
        }

        if (Input.GetKeyDown("r"))
        {
            // Add random water.
            int randomI = Random.Range(1, size - 2);
            int randomJ = Random.Range(1, size - 2);
            float randomR = Random.Range(-0.2f, -0.1f);
            h[randomI, randomJ] += randomR;
            h[randomI - 1, randomJ] += -randomR / 4.0f;
            h[randomI + 1, randomJ] += -randomR / 4.0f;
            h[randomI, randomJ - 1] += -randomR / 4.0f;
            h[randomI, randomJ + 1] += -randomR / 4.0f;
        }

        for (int l = 0; l < 8; l++)
        {
            Shallow_Wave(old_h, h, new_h, vertices);
        }

        // Store h back into X.y and recalculate normal.
        for (int i = 0; i < size; i++)
        {
            for (int j = 0; j < size; j++)
            {
                X[i * size + j].y = h[i, j];
            }
        }
        mesh.vertices = X;
        mesh.RecalculateNormals();
    }
}
