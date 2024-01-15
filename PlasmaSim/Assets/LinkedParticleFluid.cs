using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[Serializable]
public class NParticle
{
    public Transform pXform;
    public Vector3 pLastPosition;
    public Vector3 pVelocity;
    public bool isStatic = false;
    public bool[] isAttached = new bool[50];    //default array length, is set later
}
public class LinkedParticleFluid : MonoBehaviour
{
    [Range(0.1f, 3f)] public float timeScale = 1;
    public int targetFrameRate = 60;
    public bool useTargetFrameRate;
    public bool automaticSetup = false;
    public int setupCount = 10;
    public GameObject particleTemplate;
    public Vector3 setAllVelocity = Vector3.zero;
    
    public NParticle[] particles;
    public float particleSpeedModifier = 1;
    public float linkRange;
    
    public float linkRestLength;
    public float linkBreakLength = 6;
    public float linkBreakForce = 0.03f;
    public float linkAttracctLength = 4;
    
    public float springeCoef = 0.001f;
    public float springCompressCoef = 0.01f;
    public bool useDashpot = true;
    public float dashPotCoef = 0.01f;

    [Space(20)]
    public Vector3 bounds;
    public Vector3 boundPosition;
    public float wallBounceStrength = 2.5f;
    public float sphereRadius = 1;
    
    // Start is called before the first frame update
    void Start()
    {
        if (useTargetFrameRate) Application.targetFrameRate = 60;

        if (automaticSetup)
        {
            particles = new NParticle[setupCount * setupCount];

            int index = 0;

            for (int x = 0; x < setupCount; x++)
            {
                for (int y = 0; y < setupCount; y++)
                {
                    GameObject obj = Instantiate(particleTemplate,
                        new Vector3(linkRestLength * x, linkRestLength * y, 0), Quaternion.identity);
                    NParticle newP = new NParticle();
                    newP.pXform = obj.transform;
                    newP.pVelocity = setAllVelocity;
                    particles[index] = newP;
                    index++;
                }
            }
        }

        foreach (var p in particles)
        {
            p.isAttached = new bool[particles.Length];
        }
    }

    // Update is called once per frame
    void Update()
    {
        Time.timeScale = timeScale;
        
        
        //UpdateSimulation();

        

        
    }


    public void UpdateSimulation()
    {
        foreach (var p in particles)
        {
            p.pLastPosition = p.pXform.position;
            if (!p.isStatic)
            {
                p.pXform.position += p.pVelocity * Time.deltaTime * particleSpeedModifier;
                //bound check
                if (p.pXform.position.x < -bounds.x + boundPosition.x + sphereRadius || p.pXform.position.x >= bounds.x + boundPosition.x - sphereRadius) p.pVelocity.x *= -1 * wallBounceStrength;
                if (p.pXform.position.y < -bounds.y + boundPosition.y + sphereRadius || p.pXform.position.y >= bounds.y + boundPosition.y - sphereRadius) p.pVelocity.y *= -1 * wallBounceStrength;
                if (p.pXform.position.z < -bounds.z + boundPosition.z + sphereRadius || p.pXform.position.z >= bounds.z + boundPosition.z - sphereRadius) p.pVelocity.z *= -1 * wallBounceStrength;

                p.pXform.position = new Vector3(Mathf.Clamp(p.pXform.position.x, -bounds.x + boundPosition.x + sphereRadius, bounds.x + boundPosition.x - sphereRadius),
                    Mathf.Clamp(p.pXform.position.y, -bounds.y + boundPosition.y + sphereRadius, bounds.y + boundPosition.y - sphereRadius),
                    Mathf.Clamp(p.pXform.position.z, -bounds.z + boundPosition.z + sphereRadius, bounds.z + boundPosition.z - sphereRadius));
            }
        }
        
   
        
        
        foreach (var par_1 in particles)
        {
            int aIndex = 0; //index that checks through the particle's table if it's attached to the other particle
            foreach (var par_2 in particles)
            {
                if (par_1 == par_2)
                {
                    aIndex++;
                    continue;
                }


                float distance = Vector3.Distance(par_1.pXform.position, par_2.pXform.position);    //'Distance'

                
                //link check
                if (!par_1.isAttached[aIndex])
                {
                    if (distance <= linkAttracctLength) //should I move this check before the force check so force can be applied the same frame as attatchment?
                    {
                        par_1.isAttached[aIndex] = true;
                    }
                }
                
                
                
                if (par_1.isAttached[aIndex])
                {
                    Debug.Log($"CALCULATING FOR {par_1.pXform.gameObject.name} - TO - {par_2.pXform.gameObject.name}");
                
                
                    Vector3 newForce = Vector3.zero;
                
                    //Spring application
                    if (distance > linkRestLength)  //spring extension
                    {
                        float force = MathF.Abs(distance - linkRestLength) * springeCoef;
                        Vector3 dirFromBtoA = (par_2.pXform.position - par_1.pXform.position).normalized;
                        newForce += dirFromBtoA * force;
                        
                        Debug.Log($"Extension force {force}");

                        if (force > linkBreakForce) par_1.isAttached[aIndex] = false;
                    }
                    else if (distance < linkRestLength)  //spring compression
                    {
                        float force = MathF.Abs( linkRestLength - distance) * springCompressCoef;
                        Vector3 dirFromAtoB = (par_1.pXform.position - par_2.pXform.position).normalized;
                        newForce += dirFromAtoB * force;
                        
                        Debug.Log($"Compression force {force}");
                    }

                
                    //dashpot calculation. How much force is damped?
                
                    //first length
                    float len0 = distance;
                    //get the new length (length between particles after elastic force is applied)
                    float len1 = Vector3.Distance(par_1.pXform.position + newForce, par_2.pXform.position);
                    //calculate dashpot
                    float dashpotForce = dashPotCoef * Mathf.Abs(len1 - len0);
                
                    Debug.Log($"Dashpot force = {dashpotForce}");

                    if (!par_1.isStatic) par_1.pXform.position += newForce * (useDashpot ? dashpotForce : 1);
                    //if (!par_1.isStatic) par_1.pVelocity += newForce * (useDashpot ? dashpotForce : 1);
                }
                else
                {
                    
                }

                aIndex++;


            }
        }
    }
    





    void DashpotCalculation()
    {
        //viscosity coefficient
        float c = 0.5f;
        
        //damping force
        float F;
    }
    
    
    
    
    
    
    

    private void OnDrawGizmos()
    {
        
        Gizmos.color = Color.cyan;
        Gizmos.DrawWireCube(boundPosition, bounds * 2);

        foreach (var n in particles)
        {
            int aIndex = 0;
            foreach (var n2 in particles)
            {
                if (n == n2)
                {
                    aIndex++;
                    continue;
                }
                
                if (n.isAttached[aIndex])
                {
                    float fullDistance = Vector3.Distance(n.pXform.position, n2.pXform.position);

                    if (fullDistance > linkRestLength)
                    {
                        Vector3 dir = (n2.pXform.position - n.pXform.position).normalized;
                        Gizmos.color = Color.red;
                
                        Gizmos.DrawLine(n.pXform.position, n.pXform.position + dir*linkRestLength);
                        Gizmos.color = Color.blue;
                        Gizmos.DrawLine(n.pXform.position + dir*linkRestLength, n.pXform.position + dir*(fullDistance - linkRestLength));
                    }
                    else
                    {
                        Gizmos.color = Color.yellow;
                        Gizmos.DrawLine(n.pXform.position, n2.pXform.position);
                    }
                }
                
                Gizmos.color = Color.green;
                Gizmos.DrawWireSphere(n.pXform.position, sphereRadius);
                Gizmos.color = Color.magenta;
                Gizmos.DrawLine(n.pXform.position, n.pXform.position + n.pVelocity * 3);
                aIndex++;

            }
        }
        
       
        
    }
}
