using System;
using System.Collections;
using System.IO;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using Unity.Burst;
using UnityEditor;
using UnityEditor.PackageManager.UI;


[System.Serializable]
public class MeshSaveData
{
	public int[] triangles;
	public Vector3[] vertices;
	public Vector3[] normals;

	// add whatever properties of the mesh you need...

	public MeshSaveData(Mesh mesh)
	{
		this.vertices = mesh.vertices;
		this.triangles = mesh.triangles;
		this.normals = mesh.normals;
		// further properties...
	}
}

public class Marching : MonoBehaviour
{
	public bool quotePrecomputeUnquote;
	public bool doNotSimulateRendering = false;	//for testing sphere positions
	public bool useBackedUpMeshList = false;

	public bool useLinkedParticles;
	public LinkedParticleFluid fluidSim;
	
	public string fileSaveName;
	public int framesToPre = 10;
	public List<Mesh> bakedMeshes = new List<Mesh>();
	public List<Mesh> backup = new List<Mesh>();
	private List<Vector3> spherePositions = new List<Vector3>();
	private List<Vector3> sphereVelocities = new List<Vector3>();
	private int bakedMeshIndex = 0;	//loop through computed meshes
	public int bakedSimulationFPS = 10;	//loop through computed meshes

	[Space(40)]
	[SerializeField] private bool smoothLikeJazz = true;
	private List<Vector3> vertices = new List<Vector3>();
    private List<int> triangles = new List<int>();
    
    private MeshFilter meshFilter;

    private float terrainSurface = 0.5f;

    public bool debugPoints = true;
    [SerializeField] private int width = 32;
    [SerializeField] private int height = 8;
    [SerializeField] private int depth = 30;
    private float[,,] terrainMap;

    private int _configIndex = -1;

    [SerializeField] private float distanceThreshold = 5f;
    [SerializeField] private Transform sphereXform;
    [SerializeField] private Transform sphereXform_2;
    private Vector3 spherePoint;
    private Vector3 spherePoint_2;
    [SerializeField] private AnimationCurve kernel;
    [SerializeField] private Vector3 sphereVelocity;
    [SerializeField] private Vector3 sphereVelocity_2;
    [SerializeField] private Vector3 sphereAcceleration;
    [SerializeField] private float velocityLimiter = 0.5f;

    [SerializeField] private Vector3[] allPoints;
    [SerializeField] private Vector3[] allVelocities;

    // Start is called before the first frame update
    void Start()
    {
	    spherePoint = sphereXform.position;
        meshFilter = GetComponent<MeshFilter>();
        terrainMap = new float[width + 1, height + 1, depth + 1];


        Build();
        
        
    }

    public void Build()
    {
	    terrainMap = new float[width + 1, height + 1, depth + 1];
	    if (quotePrecomputeUnquote)
	    {


		    if (useLinkedParticles)
		    {
			    for (int i = 0; i < framesToPre; i++)
			    {
				    terrainMap = new float[width + 1, height + 1, depth + 1];

				    if (!doNotSimulateRendering)
				    {
					    ClearMeshData();
					    PopulateTerrainMap();
					    CreateMeshData();
					    BuildMesh();
				    }
					    
				    spherePositions.Add(spherePoint);
				    sphereVelocities.Add(sphereVelocity);
				    //spherePoint += sphereVelocity;
				    //sphereVelocity += sphereAcceleration;
				    fluidSim.UpdateSimulation();
			    }
		    }
		    else
		    {
			    if (!useBackedUpMeshList)
			    {
				    for (int i = 0; i < framesToPre; i++)
				    {
					    terrainMap = new float[width + 1, height + 1, depth + 1];
					    
					    if (!doNotSimulateRendering)
					    {
						    ClearMeshData();
						    PopulateTerrainMap();
						    CreateMeshData();
						    BuildMesh();
					    }
					    
					    spherePositions.Add(spherePoint);
					    sphereVelocities.Add(sphereVelocity);
					    //spherePoint += sphereVelocity;
					    //sphereVelocity += sphereAcceleration;


					    for (int j = 0; j < allPoints.Length; j++)
					    {
						    allPoints[j] += allVelocities[j];
					    }
				    }
			    }
			    else
			    {
				    for (int i = 0; i < backup.Count; i++)
				    {
			        
					    spherePositions.Add(spherePoint);
					    sphereVelocities.Add(sphereVelocity);
					    //spherePoint += sphereVelocity;
					    //sphereVelocity += sphereAcceleration;
				    }
		        
				    bakedMeshes.Clear();
				    bakedMeshes = backup;
			    }
		    }
		    

		    
		    
		    
		    
		    
		    
		    
		    Application.targetFrameRate = bakedSimulationFPS;
	    }
	    else
	    {
		    ClearMeshData();
		    PopulateTerrainMap();
		    CreateMeshData();
		    BuildMesh();
	    }
    }
    
    // Update is called once per frame
    void Update()
    {
	    
	    if (quotePrecomputeUnquote)
	    {
		    if (!doNotSimulateRendering)
		    {
			    meshFilter.mesh = bakedMeshes[bakedMeshIndex];
			    sphereXform.position = spherePositions[bakedMeshIndex];
			    bakedMeshIndex++;
			    if (bakedMeshIndex >= bakedMeshes.Count) bakedMeshIndex = 0;
		    }
		    else
		    {
			    sphereXform.position = spherePositions[bakedMeshIndex];
			    bakedMeshIndex++;
			    if (bakedMeshIndex >= spherePositions.Count) bakedMeshIndex = 0;
		    }
		    
		    
	    }
	    else
	    {
		    spherePoint = sphereXform.position;
		    spherePoint_2 = sphereXform_2.position;

		    if (Input.GetKeyDown(KeyCode.Space))
		    {
			    ClearMeshData();
			    PopulateTerrainMap();
			    CreateMeshData();
			    BuildMesh();
		    }
	    }
	    
    }

    void PopulateTerrainMap()
    {
	    for (int x = 0; x < width + 1; x++)
	    {
		    for (int y = 0; y < height + 1; y++)
		    {
			    for (int z = 0; z < depth + 1; z++)
			    {
				    //for terrain
				    //float thisHeight = (float) height * Mathf.PerlinNoise((float) x / 16 * 1.5f + 0.001f, (float) z / 16f * 1.5f + 0.001f);
				    
				   //terrainMap[x, y, z] = (float) y - thisHeight;
				    //Debug.Log($"new vertex value {y - thisHeight}");

				    
				    /*
				     *put radians into kernel
				     * get kernel output
				     * kernel output * velocity
				     * times that by threshold?
				     */

				    //Marching cube vertex

				    
				    
				    
				    if (!useLinkedParticles)
				    {
					    //SpheresTest(spherePoint, sphereVelocity, x, y, z);
					    //SpheresTest(spherePoint_2, sphereVelocity_2, x, y, z);
					    //SpheresTestMultiple(spherePoint, spherePoint_2, sphereVelocity, sphereVelocity_2, x, y, z);

					    Vector3[] positions = new Vector3[] {spherePoint, spherePoint_2};
					    Vector3[] velocities = new Vector3[] {sphereVelocity, sphereVelocity_2};
					    MarchPlasmasMultiple(allPoints, allVelocities, x, y, z);

				    }
				    else
				    {
					    
					    MarchPlasmasMultiple(fluidSim.GetAllParticlePositions(), fluidSim.GetAllParticleVelocities(), x, y, z);
				    }
				    
				    
				    
				    
				    
				    
				    
			    }
		    }
	    }
    }

    void SpheresTest(Vector3 pos, Vector3 vel, int xV, int yV, int zV)
    {
	    Vector3 Q = new Vector3(xV, yV, zV);	//point
	    float thisDistance = Vector3.Distance(Q, pos);
	    
	  
	    
	    Debug.Log($"Terrain is {terrainMap[xV, yV, zV]}");
	    if (terrainMap[xV, yV, zV] < distanceThreshold)
	    {
		    terrainMap[xV, yV, zV] = (float) (distanceThreshold - thisDistance);
	    }


    }
    
    void SpheresTestMultiple(Vector3 pos,Vector3 pos2, Vector3 vel, Vector3 vel2, int xV, int yV, int zV)
    {
	    Vector3 Q = new Vector3(xV, yV, zV);	//point
	    float thisDistance_1 = Vector3.Distance(Q, pos);
	    
	    float thisAngleDeg = Vector3.Angle(Vector3.Normalize(vel), (Q - pos)) - 90;
	    float thisAngleRad = thisAngleDeg * Mathf.Deg2Rad;
	    float range = (Mathf.Cos(thisAngleRad) + 1) / 2;
	    float kernelOutput = (kernel.Evaluate(range) * vel.magnitude) + 1;
	    float modThreshold_1 = distanceThreshold * kernelOutput * velocityLimiter;
	    

	    if (thisDistance_1 < modThreshold_1)
	    {
		    terrainMap[xV, yV, zV] = (float) (modThreshold_1 - thisDistance_1);
	    }
	    
	    float thisDistance_2 = Vector3.Distance(Q, pos2);
	    thisAngleDeg = Vector3.Angle(Vector3.Normalize(vel2), (Q - pos2)) - 90;
	    thisAngleRad = thisAngleDeg * Mathf.Deg2Rad;
	    range = (Mathf.Cos(thisAngleRad) + 1) / 2;
	    kernelOutput = (kernel.Evaluate(range) * vel2.magnitude) + 1;
	    float modThreshold_2 = distanceThreshold * kernelOutput * velocityLimiter;
	    
	    if (thisDistance_2 < modThreshold_2)
	    {
		    terrainMap[xV, yV, zV] = (float) (modThreshold_2 - thisDistance_2);
	    }
	    


    }
    
    
    void MarchPlasmasMultiple(Vector3[] positions, Vector3[] velocities, int xV, int yV, int zV)
    {
	    Vector3 Q = new Vector3(xV, yV, zV);	//point

	    for (int i = 0; i < positions.Length; i++)
	    {
		    float thisDistance = Vector3.Distance(Q, positions[i]);
	    
		    float thisAngleDeg = Vector3.Angle(Vector3.Normalize(velocities[i]), (Q - positions[i])) - 90;
		    float thisAngleRad = thisAngleDeg * Mathf.Deg2Rad;
		    float range = (Mathf.Cos(thisAngleRad) + 1) / 2;
		    float kernelOutput = (kernel.Evaluate(range) * velocities[i].magnitude) + 1;
		    float modThreshold = distanceThreshold * kernelOutput * velocityLimiter;
	    

		    if (thisDistance < modThreshold)
		    {
			    terrainMap[xV, yV, zV] = (float) (modThreshold - thisDistance);
		    }
	    }
	    

    }
    
    

    void CalculatePlasmaPopulation(Vector3 pos, Vector3 vel, int xV, int yV, int zV)
    {
	    Vector3 Q = new Vector3(xV, yV, zV);
				    
	    //Distance from center of sphere to this vertex
	    float thisDistance = Vector3.Distance(Q, pos);

	    float thisAngleDeg = Vector3.Angle(Vector3.Normalize(vel), (Q - pos)) - 90;	//gets in degrees
	    float thisAngleRad = thisAngleDeg * Mathf.Deg2Rad;	//converts to radians
	    float range = (Mathf.Cos(thisAngleRad) + 1) / 2;
	    Debug.Log($"RANGE: {range}");
	    float kernelOutput = (kernel.Evaluate(range) * vel.magnitude) + 1;	//amplifies kernel by particle velocity
	    Debug.Log($"KERNEL OUTPUT: {range}");
	    float modThreshold = distanceThreshold * kernelOutput * velocityLimiter;
	    //float modThreshold = distanceThreshold / kernelOutput * velocityLimiter;
	    Debug.Log("Test");
	    terrainMap[xV, yV, zV] = (float) (modThreshold - thisDistance);
	    Debug.Log($"Angle between velocity {vel} and MC vertex {Q} is deg{thisAngleDeg}/rad{thisAngleRad}. Threshold is {modThreshold}");
    }
    
    
   // float kernelOutput = kernel.Evaluate(thisAngleRad) * sphereVelocity.magnitude * velocityLimiter;
    //float kernelOutput = kernel.Evaluate(thisAngleRad) * (sphereVelocity.magnitude + 1) * velocityLimiter;
    //float kernelOutput = kernel.Evaluate(thisAngleRad) * velocityLimiter;

    void CreateMeshData()
    {
	    for (int x = 0; x < width; x++)
	    {
		    for (int y = 0; y < height; y++)
		    {
			    for (int z = 0; z < depth; z++)
			    {
				    MarchCube(new Vector3Int(x, y, z));
			    }
		    }
	    }
    }
    int GetCubeConfig(float[] cube)
    {
	    int configurationIndex = 0;
	    for (int i = 0; i < 8; i++)
	    {
		    if (cube[i] > terrainSurface)
		    {
			    configurationIndex |= 1 << i;
			    
			    
			    /*if (i == 0){
				    configurationIndex = 1;
			    }else{
				    configurationIndex += Power(i, 2);
			    }*/
			    
		    }
	    }

	    //Debug.Log("Config index " + configurationIndex);
	    return configurationIndex;
    }
    
    void MarchCube(Vector3Int position)
    {
	    
	    float[] cube = new float[8];
	    for (int i = 0; i < 8; i++)
	    {
		    cube[i] = SampleTerrain(position + CornerTable[i]);
	    }
	    
	    //Debug.Log("March cube");
	    int configIndex =  GetCubeConfig(cube);
	    
	    if (configIndex == 0 || configIndex == 255)
	    {
		    //Debug.Log("Returning");
		    return;
	    }

	    int edgeIndex = 0;
	    for (int i = 0; i < 5; i++)
	    {
		    for (int p = 0; p < 3; p++)
		    {
			    int indice = TriangleTable[configIndex, edgeIndex];

			    if (indice == -1)
				    return;

			    Vector3 vert1 = position + CornerTable[EdgeIndexes[indice, 0]];
			    Vector3 vert2 = position + CornerTable[EdgeIndexes[indice, 1]];

			    Vector3 vertPosition;
			    
			    if (smoothLikeJazz)
			    {
				    //Getting terrain values of current edge from cube array
				    float vert1Sample = cube[EdgeIndexes[indice, 0]];
				    float vert2Sample = cube[EdgeIndexes[indice, 1]];
				    
				    //Difference
				    float diff = vert2Sample - vert1Sample;
				    if (diff == 0)
					    diff = terrainSurface;
				    else
					    diff = (terrainSurface - vert1Sample) / diff;

				    //Calculate point along edge
				    vertPosition = vert1 + ((vert2 - vert1) * diff);
			    }
			    else
			    {
					vertPosition = (vert1 + vert2) / 2f;
			    }
			    
			    vertices.Add(vertPosition);
			    triangles.Add(vertices.Count - 1);
			    edgeIndex++;
			    
			    Debug.Log("March cube loop");
		    }
	    }
    }

    float SampleTerrain(Vector3Int point)
    {
	    return terrainMap[point.x, point.y, point.z];
    }
    
    void ClearMeshData()
    {
        vertices.Clear();
        triangles.Clear();
    }

    void BuildMesh()
    {
        Mesh mesh = new Mesh();
        mesh.vertices = vertices.ToArray();
        mesh.triangles = triangles.ToArray();
        mesh.RecalculateNormals();

        if (quotePrecomputeUnquote)
        {
	        bakedMeshes.Add(mesh);
        }
        else
        {
	        meshFilter.mesh = mesh;
        }
    }

    int Power(int number, int exponent){
	    int val = exponent;
	    for(int i = 1; i < number; i++){
		    val = val * exponent;   
	    }
	    return val;
    }
    
    public static float PerlinNoise3D(float x, float y, float z)
    {
	    y += 1;
	    z += 2;
	    float xy = _perlin3DFixed(x, y);
	    float xz = _perlin3DFixed(x, z);
	    float yz = _perlin3DFixed(y, z);
	    float yx = _perlin3DFixed(y, x);
	    float zx = _perlin3DFixed(z, x);
	    float zy = _perlin3DFixed(z, y);

	    return xy * xz * yz * yx * zx * zy;
    }

    static float _perlin3DFixed(float a, float b)
    {
	    return Mathf.Sin(Mathf.PI * Mathf.PerlinNoise(a, b));
    }
    
    
    // modify this for each mesh saved
    string saveFileName = "/mesh.json";
    
    void SaveMesh()
    {
	    string json = JsonUtility.ToJson(new MeshSaveData(GetComponent<MeshFilter>().sharedMesh));
	    string savePath = Application.persistentDataPath + saveFileName;

	    Debug.Log("saving mesh data to " + savePath);

	    File.WriteAllText(savePath, json);
    }

    void LoadMesh()
    {
	    string savePath = Application.persistentDataPath + saveFileName;

	    if (File.Exists(savePath))
	    {
		    string json = File.ReadAllText(savePath);

		    MeshSaveData savedMesh = JsonUtility.FromJson<MeshSaveData>(json);

		    Debug.Log("loaded mesh data from " + savePath);

		    Mesh mesh = GetComponent<MeshFilter>().sharedMesh;
		    mesh.vertices = savedMesh.vertices;
		    mesh.triangles = savedMesh.triangles;
		    mesh.normals = savedMesh.normals;
		    //...
	    }
    }

    private void OnDrawGizmosSelected()
    {
	    //Gizmos.DrawWireSphere(sphereXform_2.position, distanceThreshold);
	    //Vector3 velocityToUse;


	    for (int i = 0; i < allPoints.Length; i++)
	    {
		    Gizmos.color = Color.green;
		    Gizmos.DrawWireSphere(allPoints[i], distanceThreshold);
		    Gizmos.color = Color.blue;
		    Gizmos.DrawLine(allPoints[i], allPoints[i] + allVelocities[i] * 6);
	    }
	
	    
	    
	    
	    /*if (sphereVelocities.Count == 0)
	    {
		    velocityToUse = sphereVelocity;
	    }
	    else
	    {
		    velocityToUse = sphereVelocities[bakedMeshIndex];
	    }*/
	    
	    //Gizmos.DrawLine(sphereXform_2.position, sphereXform_2.position + velocityToUse * 10);
	    
	    if (!debugPoints) return;
	    //Gizmos.color = new Color(0.8f, 0, 0, 0.8f);
	    for (int x = 0; x < width + 1; x++)
	    {
		    for (int y = 0; y < height + 1; y++)
		    {
			    for (int z = 0; z < depth + 1; z++)
			    {
				    
				    if (Application.isPlaying)
				    {
					    float val = (terrainMap[x, y, z] - (-15)) / (3 - (-15));
					    //float val = (terrainMap[x, y, z] / distanceThreshold);
					    //Debug.Log($"COLOUR {val}");
					    Gizmos.color = new Color(val, val, val, 0.8f);
					    Gizmos.DrawSphere(new Vector3(x, y, z), 0.1f);
					    Handles.Label(new Vector3(x, y, z), System.Math.Round(terrainMap[x, y, z], 3).ToString());
				    }
				    else
				    {
					    Gizmos.color = new Color(0.8f, 0, 0, 0.8f);
					    //Gizmos.DrawSphere(new Vector3(x, y, z), 0.1f);
					    Gizmos.DrawWireCube(new Vector3(x, y, z) + new Vector3(0.5f, 0.5f, 0.5f), Vector3.one);
				    }
				    
				    //
				    //
			    }
		    }
	    }
    }

    Vector3Int[] CornerTable = new Vector3Int[8] {

        new Vector3Int(0, 0, 0),
        new Vector3Int(1, 0, 0),
        new Vector3Int(1, 1, 0),
        new Vector3Int(0, 1, 0),
        new Vector3Int(0, 0, 1),
        new Vector3Int(1, 0, 1),
        new Vector3Int(1, 1, 1),
        new Vector3Int(0, 1, 1)

    };

    int[,] EdgeIndexes = new int[12, 2] {

	    {0, 1}, {1, 2}, {3, 2}, {0, 3}, {4, 5}, {5, 6}, {7, 6}, {4, 7}, {0, 4}, {1, 5}, {2, 6}, {3, 7}

    };

    private int[,] TriangleTable = new int[,] {

	    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	    {0, 8, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	    {0, 1, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	    {1, 8, 3, 9, 8, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	    {1, 2, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	    {0, 8, 3, 1, 2, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	    {9, 2, 10, 0, 2, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	    {2, 8, 3, 2, 10, 8, 10, 9, 8, -1, -1, -1, -1, -1, -1, -1},
	    {3, 11, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	    {0, 11, 2, 8, 11, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	    {1, 9, 0, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	    {1, 11, 2, 1, 9, 11, 9, 8, 11, -1, -1, -1, -1, -1, -1, -1},
	    {3, 10, 1, 11, 10, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	    {0, 10, 1, 0, 8, 10, 8, 11, 10, -1, -1, -1, -1, -1, -1, -1},
	    {3, 9, 0, 3, 11, 9, 11, 10, 9, -1, -1, -1, -1, -1, -1, -1},
	    {9, 8, 10, 10, 8, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	    {4, 7, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	    {4, 3, 0, 7, 3, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	    {0, 1, 9, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	    {4, 1, 9, 4, 7, 1, 7, 3, 1, -1, -1, -1, -1, -1, -1, -1},
	    {1, 2, 10, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	    {3, 4, 7, 3, 0, 4, 1, 2, 10, -1, -1, -1, -1, -1, -1, -1},
	    {9, 2, 10, 9, 0, 2, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1},
	    {2, 10, 9, 2, 9, 7, 2, 7, 3, 7, 9, 4, -1, -1, -1, -1},
	    {8, 4, 7, 3, 11, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	    {11, 4, 7, 11, 2, 4, 2, 0, 4, -1, -1, -1, -1, -1, -1, -1},
	    {9, 0, 1, 8, 4, 7, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1},
	    {4, 7, 11, 9, 4, 11, 9, 11, 2, 9, 2, 1, -1, -1, -1, -1},
	    {3, 10, 1, 3, 11, 10, 7, 8, 4, -1, -1, -1, -1, -1, -1, -1},
	    {1, 11, 10, 1, 4, 11, 1, 0, 4, 7, 11, 4, -1, -1, -1, -1},
	    {4, 7, 8, 9, 0, 11, 9, 11, 10, 11, 0, 3, -1, -1, -1, -1},
	    {4, 7, 11, 4, 11, 9, 9, 11, 10, -1, -1, -1, -1, -1, -1, -1},
	    {9, 5, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	    {9, 5, 4, 0, 8, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	    {0, 5, 4, 1, 5, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	    {8, 5, 4, 8, 3, 5, 3, 1, 5, -1, -1, -1, -1, -1, -1, -1},
	    {1, 2, 10, 9, 5, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	    {3, 0, 8, 1, 2, 10, 4, 9, 5, -1, -1, -1, -1, -1, -1, -1},
	    {5, 2, 10, 5, 4, 2, 4, 0, 2, -1, -1, -1, -1, -1, -1, -1},
	    {2, 10, 5, 3, 2, 5, 3, 5, 4, 3, 4, 8, -1, -1, -1, -1},
	    {9, 5, 4, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	    {0, 11, 2, 0, 8, 11, 4, 9, 5, -1, -1, -1, -1, -1, -1, -1},
	    {0, 5, 4, 0, 1, 5, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1},
	    {2, 1, 5, 2, 5, 8, 2, 8, 11, 4, 8, 5, -1, -1, -1, -1},
	    {10, 3, 11, 10, 1, 3, 9, 5, 4, -1, -1, -1, -1, -1, -1, -1},
	    {4, 9, 5, 0, 8, 1, 8, 10, 1, 8, 11, 10, -1, -1, -1, -1},
	    {5, 4, 0, 5, 0, 11, 5, 11, 10, 11, 0, 3, -1, -1, -1, -1},
	    {5, 4, 8, 5, 8, 10, 10, 8, 11, -1, -1, -1, -1, -1, -1, -1},
	    {9, 7, 8, 5, 7, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	    {9, 3, 0, 9, 5, 3, 5, 7, 3, -1, -1, -1, -1, -1, -1, -1},
	    {0, 7, 8, 0, 1, 7, 1, 5, 7, -1, -1, -1, -1, -1, -1, -1},
	    {1, 5, 3, 3, 5, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	    {9, 7, 8, 9, 5, 7, 10, 1, 2, -1, -1, -1, -1, -1, -1, -1},
	    {10, 1, 2, 9, 5, 0, 5, 3, 0, 5, 7, 3, -1, -1, -1, -1},
	    {8, 0, 2, 8, 2, 5, 8, 5, 7, 10, 5, 2, -1, -1, -1, -1},
	    {2, 10, 5, 2, 5, 3, 3, 5, 7, -1, -1, -1, -1, -1, -1, -1},
	    {7, 9, 5, 7, 8, 9, 3, 11, 2, -1, -1, -1, -1, -1, -1, -1},
	    {9, 5, 7, 9, 7, 2, 9, 2, 0, 2, 7, 11, -1, -1, -1, -1},
	    {2, 3, 11, 0, 1, 8, 1, 7, 8, 1, 5, 7, -1, -1, -1, -1},
	    {11, 2, 1, 11, 1, 7, 7, 1, 5, -1, -1, -1, -1, -1, -1, -1},
	    {9, 5, 8, 8, 5, 7, 10, 1, 3, 10, 3, 11, -1, -1, -1, -1},
	    {5, 7, 0, 5, 0, 9, 7, 11, 0, 1, 0, 10, 11, 10, 0, -1},
	    {11, 10, 0, 11, 0, 3, 10, 5, 0, 8, 0, 7, 5, 7, 0, -1},
	    {11, 10, 5, 7, 11, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	    {10, 6, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	    {0, 8, 3, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	    {9, 0, 1, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	    {1, 8, 3, 1, 9, 8, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1},
	    {1, 6, 5, 2, 6, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	    {1, 6, 5, 1, 2, 6, 3, 0, 8, -1, -1, -1, -1, -1, -1, -1},
	    {9, 6, 5, 9, 0, 6, 0, 2, 6, -1, -1, -1, -1, -1, -1, -1},
	    {5, 9, 8, 5, 8, 2, 5, 2, 6, 3, 2, 8, -1, -1, -1, -1},
	    {2, 3, 11, 10, 6, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	    {11, 0, 8, 11, 2, 0, 10, 6, 5, -1, -1, -1, -1, -1, -1, -1},
	    {0, 1, 9, 2, 3, 11, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1},
	    {5, 10, 6, 1, 9, 2, 9, 11, 2, 9, 8, 11, -1, -1, -1, -1},
	    {6, 3, 11, 6, 5, 3, 5, 1, 3, -1, -1, -1, -1, -1, -1, -1},
	    {0, 8, 11, 0, 11, 5, 0, 5, 1, 5, 11, 6, -1, -1, -1, -1},
	    {3, 11, 6, 0, 3, 6, 0, 6, 5, 0, 5, 9, -1, -1, -1, -1},
	    {6, 5, 9, 6, 9, 11, 11, 9, 8, -1, -1, -1, -1, -1, -1, -1},
	    {5, 10, 6, 4, 7, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	    {4, 3, 0, 4, 7, 3, 6, 5, 10, -1, -1, -1, -1, -1, -1, -1},
	    {1, 9, 0, 5, 10, 6, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1},
	    {10, 6, 5, 1, 9, 7, 1, 7, 3, 7, 9, 4, -1, -1, -1, -1},
	    {6, 1, 2, 6, 5, 1, 4, 7, 8, -1, -1, -1, -1, -1, -1, -1},
	    {1, 2, 5, 5, 2, 6, 3, 0, 4, 3, 4, 7, -1, -1, -1, -1},
	    {8, 4, 7, 9, 0, 5, 0, 6, 5, 0, 2, 6, -1, -1, -1, -1},
	    {7, 3, 9, 7, 9, 4, 3, 2, 9, 5, 9, 6, 2, 6, 9, -1},
	    {3, 11, 2, 7, 8, 4, 10, 6, 5, -1, -1, -1, -1, -1, -1, -1},
	    {5, 10, 6, 4, 7, 2, 4, 2, 0, 2, 7, 11, -1, -1, -1, -1},
	    {0, 1, 9, 4, 7, 8, 2, 3, 11, 5, 10, 6, -1, -1, -1, -1},
	    {9, 2, 1, 9, 11, 2, 9, 4, 11, 7, 11, 4, 5, 10, 6, -1},
	    {8, 4, 7, 3, 11, 5, 3, 5, 1, 5, 11, 6, -1, -1, -1, -1},
	    {5, 1, 11, 5, 11, 6, 1, 0, 11, 7, 11, 4, 0, 4, 11, -1},
	    {0, 5, 9, 0, 6, 5, 0, 3, 6, 11, 6, 3, 8, 4, 7, -1},
	    {6, 5, 9, 6, 9, 11, 4, 7, 9, 7, 11, 9, -1, -1, -1, -1},
	    {10, 4, 9, 6, 4, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	    {4, 10, 6, 4, 9, 10, 0, 8, 3, -1, -1, -1, -1, -1, -1, -1},
	    {10, 0, 1, 10, 6, 0, 6, 4, 0, -1, -1, -1, -1, -1, -1, -1},
	    {8, 3, 1, 8, 1, 6, 8, 6, 4, 6, 1, 10, -1, -1, -1, -1},
	    {1, 4, 9, 1, 2, 4, 2, 6, 4, -1, -1, -1, -1, -1, -1, -1},
	    {3, 0, 8, 1, 2, 9, 2, 4, 9, 2, 6, 4, -1, -1, -1, -1},
	    {0, 2, 4, 4, 2, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	    {8, 3, 2, 8, 2, 4, 4, 2, 6, -1, -1, -1, -1, -1, -1, -1},
	    {10, 4, 9, 10, 6, 4, 11, 2, 3, -1, -1, -1, -1, -1, -1, -1},
	    {0, 8, 2, 2, 8, 11, 4, 9, 10, 4, 10, 6, -1, -1, -1, -1},
	    {3, 11, 2, 0, 1, 6, 0, 6, 4, 6, 1, 10, -1, -1, -1, -1},
	    {6, 4, 1, 6, 1, 10, 4, 8, 1, 2, 1, 11, 8, 11, 1, -1},
	    {9, 6, 4, 9, 3, 6, 9, 1, 3, 11, 6, 3, -1, -1, -1, -1},
	    {8, 11, 1, 8, 1, 0, 11, 6, 1, 9, 1, 4, 6, 4, 1, -1},
	    {3, 11, 6, 3, 6, 0, 0, 6, 4, -1, -1, -1, -1, -1, -1, -1},
	    {6, 4, 8, 11, 6, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	    {7, 10, 6, 7, 8, 10, 8, 9, 10, -1, -1, -1, -1, -1, -1, -1},
	    {0, 7, 3, 0, 10, 7, 0, 9, 10, 6, 7, 10, -1, -1, -1, -1},
	    {10, 6, 7, 1, 10, 7, 1, 7, 8, 1, 8, 0, -1, -1, -1, -1},
	    {10, 6, 7, 10, 7, 1, 1, 7, 3, -1, -1, -1, -1, -1, -1, -1},
	    {1, 2, 6, 1, 6, 8, 1, 8, 9, 8, 6, 7, -1, -1, -1, -1},
	    {2, 6, 9, 2, 9, 1, 6, 7, 9, 0, 9, 3, 7, 3, 9, -1},
	    {7, 8, 0, 7, 0, 6, 6, 0, 2, -1, -1, -1, -1, -1, -1, -1},
	    {7, 3, 2, 6, 7, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	    {2, 3, 11, 10, 6, 8, 10, 8, 9, 8, 6, 7, -1, -1, -1, -1},
	    {2, 0, 7, 2, 7, 11, 0, 9, 7, 6, 7, 10, 9, 10, 7, -1},
	    {1, 8, 0, 1, 7, 8, 1, 10, 7, 6, 7, 10, 2, 3, 11, -1},
	    {11, 2, 1, 11, 1, 7, 10, 6, 1, 6, 7, 1, -1, -1, -1, -1},
	    {8, 9, 6, 8, 6, 7, 9, 1, 6, 11, 6, 3, 1, 3, 6, -1},
	    {0, 9, 1, 11, 6, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	    {7, 8, 0, 7, 0, 6, 3, 11, 0, 11, 6, 0, -1, -1, -1, -1},
	    {7, 11, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	    {7, 6, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	    {3, 0, 8, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	    {0, 1, 9, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	    {8, 1, 9, 8, 3, 1, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1},
	    {10, 1, 2, 6, 11, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	    {1, 2, 10, 3, 0, 8, 6, 11, 7, -1, -1, -1, -1, -1, -1, -1},
	    {2, 9, 0, 2, 10, 9, 6, 11, 7, -1, -1, -1, -1, -1, -1, -1},
	    {6, 11, 7, 2, 10, 3, 10, 8, 3, 10, 9, 8, -1, -1, -1, -1},
	    {7, 2, 3, 6, 2, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	    {7, 0, 8, 7, 6, 0, 6, 2, 0, -1, -1, -1, -1, -1, -1, -1},
	    {2, 7, 6, 2, 3, 7, 0, 1, 9, -1, -1, -1, -1, -1, -1, -1},
	    {1, 6, 2, 1, 8, 6, 1, 9, 8, 8, 7, 6, -1, -1, -1, -1},
	    {10, 7, 6, 10, 1, 7, 1, 3, 7, -1, -1, -1, -1, -1, -1, -1},
	    {10, 7, 6, 1, 7, 10, 1, 8, 7, 1, 0, 8, -1, -1, -1, -1},
	    {0, 3, 7, 0, 7, 10, 0, 10, 9, 6, 10, 7, -1, -1, -1, -1},
	    {7, 6, 10, 7, 10, 8, 8, 10, 9, -1, -1, -1, -1, -1, -1, -1},
	    {6, 8, 4, 11, 8, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	    {3, 6, 11, 3, 0, 6, 0, 4, 6, -1, -1, -1, -1, -1, -1, -1},
	    {8, 6, 11, 8, 4, 6, 9, 0, 1, -1, -1, -1, -1, -1, -1, -1},
	    {9, 4, 6, 9, 6, 3, 9, 3, 1, 11, 3, 6, -1, -1, -1, -1},
	    {6, 8, 4, 6, 11, 8, 2, 10, 1, -1, -1, -1, -1, -1, -1, -1},
	    {1, 2, 10, 3, 0, 11, 0, 6, 11, 0, 4, 6, -1, -1, -1, -1},
	    {4, 11, 8, 4, 6, 11, 0, 2, 9, 2, 10, 9, -1, -1, -1, -1},
	    {10, 9, 3, 10, 3, 2, 9, 4, 3, 11, 3, 6, 4, 6, 3, -1},
	    {8, 2, 3, 8, 4, 2, 4, 6, 2, -1, -1, -1, -1, -1, -1, -1},
	    {0, 4, 2, 4, 6, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	    {1, 9, 0, 2, 3, 4, 2, 4, 6, 4, 3, 8, -1, -1, -1, -1},
	    {1, 9, 4, 1, 4, 2, 2, 4, 6, -1, -1, -1, -1, -1, -1, -1},
	    {8, 1, 3, 8, 6, 1, 8, 4, 6, 6, 10, 1, -1, -1, -1, -1},
	    {10, 1, 0, 10, 0, 6, 6, 0, 4, -1, -1, -1, -1, -1, -1, -1},
	    {4, 6, 3, 4, 3, 8, 6, 10, 3, 0, 3, 9, 10, 9, 3, -1},
	    {10, 9, 4, 6, 10, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	    {4, 9, 5, 7, 6, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	    {0, 8, 3, 4, 9, 5, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1},
	    {5, 0, 1, 5, 4, 0, 7, 6, 11, -1, -1, -1, -1, -1, -1, -1},
	    {11, 7, 6, 8, 3, 4, 3, 5, 4, 3, 1, 5, -1, -1, -1, -1},
	    {9, 5, 4, 10, 1, 2, 7, 6, 11, -1, -1, -1, -1, -1, -1, -1},
	    {6, 11, 7, 1, 2, 10, 0, 8, 3, 4, 9, 5, -1, -1, -1, -1},
	    {7, 6, 11, 5, 4, 10, 4, 2, 10, 4, 0, 2, -1, -1, -1, -1},
	    {3, 4, 8, 3, 5, 4, 3, 2, 5, 10, 5, 2, 11, 7, 6, -1},
	    {7, 2, 3, 7, 6, 2, 5, 4, 9, -1, -1, -1, -1, -1, -1, -1},
	    {9, 5, 4, 0, 8, 6, 0, 6, 2, 6, 8, 7, -1, -1, -1, -1},
	    {3, 6, 2, 3, 7, 6, 1, 5, 0, 5, 4, 0, -1, -1, -1, -1},
	    {6, 2, 8, 6, 8, 7, 2, 1, 8, 4, 8, 5, 1, 5, 8, -1},
	    {9, 5, 4, 10, 1, 6, 1, 7, 6, 1, 3, 7, -1, -1, -1, -1},
	    {1, 6, 10, 1, 7, 6, 1, 0, 7, 8, 7, 0, 9, 5, 4, -1},
	    {4, 0, 10, 4, 10, 5, 0, 3, 10, 6, 10, 7, 3, 7, 10, -1},
	    {7, 6, 10, 7, 10, 8, 5, 4, 10, 4, 8, 10, -1, -1, -1, -1},
	    {6, 9, 5, 6, 11, 9, 11, 8, 9, -1, -1, -1, -1, -1, -1, -1},
	    {3, 6, 11, 0, 6, 3, 0, 5, 6, 0, 9, 5, -1, -1, -1, -1},
	    {0, 11, 8, 0, 5, 11, 0, 1, 5, 5, 6, 11, -1, -1, -1, -1},
	    {6, 11, 3, 6, 3, 5, 5, 3, 1, -1, -1, -1, -1, -1, -1, -1},
	    {1, 2, 10, 9, 5, 11, 9, 11, 8, 11, 5, 6, -1, -1, -1, -1},
	    {0, 11, 3, 0, 6, 11, 0, 9, 6, 5, 6, 9, 1, 2, 10, -1},
	    {11, 8, 5, 11, 5, 6, 8, 0, 5, 10, 5, 2, 0, 2, 5, -1},
	    {6, 11, 3, 6, 3, 5, 2, 10, 3, 10, 5, 3, -1, -1, -1, -1},
	    {5, 8, 9, 5, 2, 8, 5, 6, 2, 3, 8, 2, -1, -1, -1, -1},
	    {9, 5, 6, 9, 6, 0, 0, 6, 2, -1, -1, -1, -1, -1, -1, -1},
	    {1, 5, 8, 1, 8, 0, 5, 6, 8, 3, 8, 2, 6, 2, 8, -1},
	    {1, 5, 6, 2, 1, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	    {1, 3, 6, 1, 6, 10, 3, 8, 6, 5, 6, 9, 8, 9, 6, -1},
	    {10, 1, 0, 10, 0, 6, 9, 5, 0, 5, 6, 0, -1, -1, -1, -1},
	    {0, 3, 8, 5, 6, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	    {10, 5, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	    {11, 5, 10, 7, 5, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	    {11, 5, 10, 11, 7, 5, 8, 3, 0, -1, -1, -1, -1, -1, -1, -1},
	    {5, 11, 7, 5, 10, 11, 1, 9, 0, -1, -1, -1, -1, -1, -1, -1},
	    {10, 7, 5, 10, 11, 7, 9, 8, 1, 8, 3, 1, -1, -1, -1, -1},
	    {11, 1, 2, 11, 7, 1, 7, 5, 1, -1, -1, -1, -1, -1, -1, -1},
	    {0, 8, 3, 1, 2, 7, 1, 7, 5, 7, 2, 11, -1, -1, -1, -1},
	    {9, 7, 5, 9, 2, 7, 9, 0, 2, 2, 11, 7, -1, -1, -1, -1},
	    {7, 5, 2, 7, 2, 11, 5, 9, 2, 3, 2, 8, 9, 8, 2, -1},
	    {2, 5, 10, 2, 3, 5, 3, 7, 5, -1, -1, -1, -1, -1, -1, -1},
	    {8, 2, 0, 8, 5, 2, 8, 7, 5, 10, 2, 5, -1, -1, -1, -1},
	    {9, 0, 1, 5, 10, 3, 5, 3, 7, 3, 10, 2, -1, -1, -1, -1},
	    {9, 8, 2, 9, 2, 1, 8, 7, 2, 10, 2, 5, 7, 5, 2, -1},
	    {1, 3, 5, 3, 7, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	    {0, 8, 7, 0, 7, 1, 1, 7, 5, -1, -1, -1, -1, -1, -1, -1},
	    {9, 0, 3, 9, 3, 5, 5, 3, 7, -1, -1, -1, -1, -1, -1, -1},
	    {9, 8, 7, 5, 9, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	    {5, 8, 4, 5, 10, 8, 10, 11, 8, -1, -1, -1, -1, -1, -1, -1},
	    {5, 0, 4, 5, 11, 0, 5, 10, 11, 11, 3, 0, -1, -1, -1, -1},
	    {0, 1, 9, 8, 4, 10, 8, 10, 11, 10, 4, 5, -1, -1, -1, -1},
	    {10, 11, 4, 10, 4, 5, 11, 3, 4, 9, 4, 1, 3, 1, 4, -1},
	    {2, 5, 1, 2, 8, 5, 2, 11, 8, 4, 5, 8, -1, -1, -1, -1},
	    {0, 4, 11, 0, 11, 3, 4, 5, 11, 2, 11, 1, 5, 1, 11, -1},
	    {0, 2, 5, 0, 5, 9, 2, 11, 5, 4, 5, 8, 11, 8, 5, -1},
	    {9, 4, 5, 2, 11, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	    {2, 5, 10, 3, 5, 2, 3, 4, 5, 3, 8, 4, -1, -1, -1, -1},
	    {5, 10, 2, 5, 2, 4, 4, 2, 0, -1, -1, -1, -1, -1, -1, -1},
	    {3, 10, 2, 3, 5, 10, 3, 8, 5, 4, 5, 8, 0, 1, 9, -1},
	    {5, 10, 2, 5, 2, 4, 1, 9, 2, 9, 4, 2, -1, -1, -1, -1},
	    {8, 4, 5, 8, 5, 3, 3, 5, 1, -1, -1, -1, -1, -1, -1, -1},
	    {0, 4, 5, 1, 0, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	    {8, 4, 5, 8, 5, 3, 9, 0, 5, 0, 3, 5, -1, -1, -1, -1},
	    {9, 4, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	    {4, 11, 7, 4, 9, 11, 9, 10, 11, -1, -1, -1, -1, -1, -1, -1},
	    {0, 8, 3, 4, 9, 7, 9, 11, 7, 9, 10, 11, -1, -1, -1, -1},
	    {1, 10, 11, 1, 11, 4, 1, 4, 0, 7, 4, 11, -1, -1, -1, -1},
	    {3, 1, 4, 3, 4, 8, 1, 10, 4, 7, 4, 11, 10, 11, 4, -1},
	    {4, 11, 7, 9, 11, 4, 9, 2, 11, 9, 1, 2, -1, -1, -1, -1},
	    {9, 7, 4, 9, 11, 7, 9, 1, 11, 2, 11, 1, 0, 8, 3, -1},
	    {11, 7, 4, 11, 4, 2, 2, 4, 0, -1, -1, -1, -1, -1, -1, -1},
	    {11, 7, 4, 11, 4, 2, 8, 3, 4, 3, 2, 4, -1, -1, -1, -1},
	    {2, 9, 10, 2, 7, 9, 2, 3, 7, 7, 4, 9, -1, -1, -1, -1},
	    {9, 10, 7, 9, 7, 4, 10, 2, 7, 8, 7, 0, 2, 0, 7, -1},
	    {3, 7, 10, 3, 10, 2, 7, 4, 10, 1, 10, 0, 4, 0, 10, -1},
	    {1, 10, 2, 8, 7, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	    {4, 9, 1, 4, 1, 7, 7, 1, 3, -1, -1, -1, -1, -1, -1, -1},
	    {4, 9, 1, 4, 1, 7, 0, 8, 1, 8, 7, 1, -1, -1, -1, -1},
	    {4, 0, 3, 7, 4, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	    {4, 8, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	    {9, 10, 8, 10, 11, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	    {3, 0, 9, 3, 9, 11, 11, 9, 10, -1, -1, -1, -1, -1, -1, -1},
	    {0, 1, 10, 0, 10, 8, 8, 10, 11, -1, -1, -1, -1, -1, -1, -1},
	    {3, 1, 10, 11, 3, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	    {1, 2, 11, 1, 11, 9, 9, 11, 8, -1, -1, -1, -1, -1, -1, -1},
	    {3, 0, 9, 3, 9, 11, 1, 2, 9, 2, 11, 9, -1, -1, -1, -1},
	    {0, 2, 11, 8, 0, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	    {3, 2, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	    {2, 3, 8, 2, 8, 10, 10, 8, 9, -1, -1, -1, -1, -1, -1, -1},
	    {9, 10, 2, 0, 9, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	    {2, 3, 8, 2, 8, 10, 0, 1, 8, 1, 10, 8, -1, -1, -1, -1},
	    {1, 10, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	    {1, 3, 8, 9, 1, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	    {0, 9, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	    {0, 3, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}

    };
}
