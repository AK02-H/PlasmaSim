using System;
using System.Collections;
using System.Collections.Generic;
using System.Data;
using UnityEngine;

[Serializable]
public class SphereData
{
    public Vector3 startPos;
    public Vector3 center;
    public Vector3 velocity;
    public float radius;
    public float mass;
    public Color renderCol = new Color(1, 1, 1, 1);
}

[Serializable]
public class PlaneData
{
    public Vector3 normal;
    public Vector3 arbitrary;
}

public struct pairBF
{
    public bool b;
    public float f;
}

public struct pairFF
{
    public float f1;
    public float f2;

}

public class SphereManager : MonoBehaviour
{

    public bool activeSimulation = true;

    [Range(1, 120)] public int targetFrameRate = 30;
    [Range(0.01f, 30)] public float timeScale = 1;
    [Range(1f, 30)] public float globalSpeedMultiplier = 1;
    
    [SerializeField] private SphereData[] sph = new SphereData[2];
    [SerializeField] private PlaneData[] pln = new PlaneData[2];

    private bool collisionHappened = false;
    private Vector3 collisionPosition;
    private Vector3 v1;
    private Vector3 v2;
    private Vector3 v3;
    private float f1;

    private int curFrame = 0;
    public int stopAtFrame = 10;
    
    void Start()
    {
        Application.targetFrameRate = targetFrameRate;
        
        //Debug.Log($"INITIAL DISTANCE OF 2 SPHERES: {Vector3.Distance(sph[0].center, sph[1].center)}");
    }


    private void Update()   //does it matter if this is fixed update
    {
        Time.timeScale = timeScale;

        if (curFrame == stopAtFrame)
        {
            //Debug.Break();
        }

        curFrame++;
        if (activeSimulation)
        {




            #region collision planes

            {




                foreach (SphereData sphere in sph)
                {
                    //sphere against plane
                    foreach (PlaneData plane in pln)
                    {
                        
                        float angle = Vector3.Angle(-sphere.velocity, plane.normal);
                        if (angle < 90) //sphere is headed towards plane
                        {

                            Vector3 P = sphere.center - plane.arbitrary;
                            Debug.DrawLine(plane.arbitrary, sphere.center, Color.magenta);
                            
                            Debug.Log($"P: {P}");

                            float q1 = Vector3.Angle(plane.normal, P);
                            float q2 = 90 - q1;
                            
                            Debug.Log($"Q1: {q1}");
                            Debug.Log($"Q2: {q2}");

                            float d = Mathf.Sin(q2 * Mathf.Deg2Rad) * Vector3.Magnitude(P);
                            float s = Vector3.Angle(sphere.velocity, -plane.normal);
                            
                            Debug.Log($"d: {d}");
                            Debug.Log($"s: {s}");

                            float vcLength = (d - sphere.radius) / Mathf.Cos(s * Mathf.Deg2Rad);
                            
                            Debug.Log($"vcLength: {vcLength}");
                           // Debug.Log($"COS PROPORTION: {Mathf.Cos(s * Mathf.Deg2Rad)}");

                            if (vcLength <= Vector3.Magnitude(sphere.velocity))
                            {
                                Debug.Log("WILL HIT");


                                Vector3 vc = (vcLength * sphere.velocity) / Vector3.Magnitude(sphere.velocity);
                                
                                Debug.Log($"Full velocity: {(sphere.velocity)}");
                                Debug.Log($"vc: {vc}");
                                Debug.Log($"COS PROPORTION: {(vcLength) / Vector3.Magnitude(sphere.velocity)}");
                                
                                sphere.center += vc;
                                
                                //for rebound: use remainder of vc fraction to position sphere with new velocity after collision
                                
                                
                                //check if this is working properly
                                
                                
                                //activeSimulation = false;
                                Vector3 vHatNeg = (sphere.velocity / Vector3.Magnitude(sphere.velocity));
                                Vector3 vHatPos = (2 * plane.normal) * (Vector3.Dot(plane.normal, -vHatNeg)) + vHatNeg;
                                Vector3 vPos = vHatPos * Vector3.Magnitude(sphere.velocity);
                                sphere.velocity = vPos;
                                sphere.center += sphere.velocity * (1 - Mathf.Cos(s * Mathf.Deg2Rad));    //change this fraction later
                                Debug.Log($"ADDITIONAL MOVE AFTER COLLISION: {sphere.velocity * (1 - (vcLength) / Vector3.Magnitude(sphere.velocity))}");
                                return;
                            }
                        }
                        else
                        {
                            Debug.Log("SPHERE WILL NOT HIT PLANE");
                        }
                    
                    }

                }


                //return;


            }

            #endregion



            //test



            foreach (SphereData sphOne in sph)
            {

                foreach (SphereData sphNext in sph)
                {
                    if (sphOne == sphNext) continue;

                    SphereData s1 = sphOne;
                    SphereData s2 = sphNext;

                    

                    #region collision spheres



                    float dxp = s1.center.x - s2.center.x;
                    float dyp = s1.center.y - s2.center.y;
                    float dzp = s1.center.z - s2.center.z;

                    float dxv = s1.velocity.x - s2.velocity.x;
                    float dyv = s1.velocity.y - s2.velocity.y;
                    float dzv = s1.velocity.z - s2.velocity.z;

                    float qA = (dxv * dxv) + (dyv * dyv) + (dzv * dzv);
                    float qB = (2 * dxp * dxv) + (2 * dyp * dyv) + (2 * dzp * dzv);
                    float qC = (dxp * dxp) + (dyp * dyp) + (dzp * dzp) -
                               ((s1.radius + s2.radius) * (s1.radius + s2.radius));

                    float finalT = 0;

                    pairFF qSol = QuadraticSolve(qA, qB, qC);

                    if (!in01(qSol.f1) && !in01(qSol.f2)) //if none of the solutions are in range
                    {

                        foreach (var s in sph)
                        {
                            s.center += s.velocity; //will there be issues locking this to fixed dt
                        }

                        //skip rest of function if there is no collision
                        return;
                    }
                    else if (in01(qSol.f1) || in01(qSol.f2)) //one of the solutions between 0 and 1
                    {

                        if (in01(qSol.f1) && in01(qSol.f2)) //if both t values between 0 and 1, use the smaller one
                        {
                            finalT = Mathf.Min(qSol.f1, qSol.f2);
                        }
                        else if (in01(qSol.f1))
                        {
                            finalT = qSol.f1;
                        }
                        else if (in01(qSol.f2))
                        {
                            finalT = qSol.f2;
                        }

                        foreach (var s in sph)
                        {
                            s.center += s.velocity * finalT;
                        }


                        Debug.Log($"Final T: {finalT}");


                        //activeSimulation = false;
                    }
                    else
                    {
                        Debug.Log($"ELSE CASE: {qSol.f1}, {qSol.f2}");
                    }




                    #endregion

                    #region collision reaction


                    {
                        
                        //Collision reaction

                        Vector3 dFroms1tos2 = (s2.center - s1.center);
                        float dFroms1tos2_mag = Vector3.Magnitude(dFroms1tos2);
                        Vector3 dFroms1tos2_unit = Vector3.Normalize(s2.center - s1.center);
                        float distanceBetweenSpheres = s1.radius;

                        Vector3 positionOfCollision = s1.center + (dFroms1tos2_unit * distanceBetweenSpheres);

                        v1 = s1.center;
                        v2 = dFroms1tos2_unit;
                        v3 = dFroms1tos2_unit * distanceBetweenSpheres;
                        f1 = distanceBetweenSpheres;

                        //Vector3 collisionPoint = (s2.radius + s1.radius) * ( /Vector3.Magnitude((s2.center - s1.center))) + s1.center; //?
                        collisionPosition = positionOfCollision;

                        Vector3 v1_neg, v2_neg, v1_plus, v2_plus;

                        v1_neg = s1.velocity;
                        v2_neg = s2.velocity;

                        Vector3 totalMomentum = v1_neg * s1.mass + v2_neg * s2.mass;
                        Debug.Log($"TOTAL MOMENTUM: {totalMomentum}");

                        float q1 = Vector3.Angle((s2.center - s1.center), s1.velocity); //calculated in degrees, must be converted to radians for cos calculation
                        Debug.Log($"ANGLE 1: {q1}");
                        Vector3 z21 = (s2.center - s1.center) / (Vector3.Magnitude(s2.center - s1.center)); // unit vector of force of sphere 2 upon sphere 1

                        float angle1Cos = Mathf.Cos(q1 * Mathf.Deg2Rad);
                        float sph1VMag = Vector3.Magnitude(s1.velocity);
                        float b21 = (angle1Cos * sph1VMag * s1.mass) / s2.mass;

                        float q2 = Vector3.Angle((s1.center - s2.center), s2.velocity); //reversed
                        Debug.Log($"ANGLE 2: {q2}");
                        Vector3 z12 = (s1.center - s2.center) / (Vector3.Magnitude(s1.center - s2.center)); // unit vector of force of sphere 1 upon sphere 2
                        float angle2Cos = Mathf.Cos(q2 * Mathf.Deg2Rad);
                        float sph2VMag = Vector3.Magnitude(s2.velocity);
                        float b12 = (angle2Cos * sph2VMag * s2.mass) / s1.mass; //strength of force of sphere 1 upon sphere 2
                        Debug.DrawLine(s1.center, s1.center + s1.velocity * 10, Color.red);
                        Debug.DrawLine(s1.center, s2.center, Color.red);

                        Debug.DrawLine(s2.center, s2.center + s2.velocity * 10, Color.cyan);
                        Debug.DrawLine(s2.center, s1.center, Color.cyan);

                        v1_plus = (b12 * z12) + ((v1_neg * s1.mass) - (b21 * z21));
                        v2_plus = (b21 * z21) + ((v2_neg * s2.mass) - (b12 * z12));

                        Debug.Log($"Final new velocity of sphere 1: {v1_plus}");
                        Debug.Log($"Final new velocity of sphere 2: {v2_plus}");

                        s1.velocity = v1_plus;
                        s2.velocity = v2_plus;

                        Debug.Log($"New total momentum is {v1_plus * s1.mass + v2_plus * s2.mass}. Original total momentum was {totalMomentum}.");
                        Debug.Log($"Leftover momentum is {totalMomentum - ((v1_plus * s1.mass) + (v2_plus * s2.mass))}");

                        s1.center += s1.velocity * (1 - finalT);
                        s2.center += s2.velocity * (1 - finalT);

                        return;

                    }

                    #endregion

                }
                
                

            }
            
            //Only called when no collision
            foreach (var s in sph)
            {
                s.center += s.velocity; //will there be issues locking this to fixed dt
            }

            

        }
    }

    private void OnDrawGizmos()
    {
        foreach (var s in sph)
        {
            Gizmos.color = s.renderCol;
             
            Gizmos.DrawSphere(s.center, s.radius);
            Gizmos.color = Color.white;
            Gizmos.DrawWireSphere(s.center, s.radius * 1.001f);
        }

        if (collisionHappened)
        {
            Gizmos.color = Color.green;
            Gizmos.DrawSphere(collisionPosition, 0.5f);
            
            Debug.DrawLine(v1, v1 + (v2 * f1), Color.cyan, 100);
        }
        
        foreach (var s in sph)
        {
            Gizmos.color = Color.green;
            Debug.DrawRay(s.center, Vector3.Normalize(s.velocity) * 50);
        }

        foreach (var plane in pln)
        {
            Gizmos.color = Color.red;
            Gizmos.DrawLine(plane.arbitrary, plane.arbitrary + plane.normal * 50);
        }
    }

    private pairFF QuadraticSolve(float A, float B, float C)
    {
        //Debug.Log($"QUADRATIC WALKTHROUGH");
        
        float toRoot = ((B * B) - (4 * A * C));
        
        if (toRoot < 0)
        {
            pairFF solution = new pairFF();
            solution.f1 = -10;
            solution.f2 = -10;

            Debug.LogWarning("NEGATIVE QUADRATIC ROOT. RETURNING ERRONEOUS VALUES.");
            return solution;
        }
        else
        {
             // b^2 - 4ac is greater than 0, B has 2 solutions 
            
            float rooted = Mathf.Sqrt(toRoot);
            //float bNeg = B * -1;

            float bottomHalf = 2 * A;
            float topHalf1 = -B + rooted;
            float topHalf2 = -B - rooted;

            float sol1 = topHalf1 / bottomHalf;
            float sol2 = topHalf2 / bottomHalf;

            pairFF solution = new pairFF();
            solution.f1 = sol1;
            solution.f2 = sol2;
            return solution;
        }
        

    }

    bool in01(float a)  //inclusive?
    {
        if(a > 0 && a <= 1)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    
}
