using UnityEngine;
using System.Collections.Generic;

[RequireComponent(typeof(MeshFilter))]
[RequireComponent(typeof(MeshRenderer))]
public class PuruPuru : MonoBehaviour
{
    [Header("物理パラメータ")]
    [SerializeField] private float springConstant = 50f;
    [SerializeField] private float damping = 5f;
    [SerializeField] private float mass = 1f;
    [SerializeField] private float gravity = -9.81f;
    
    [Header("プリン特性")]
    [SerializeField] private float surfaceTension = 2f;
    [SerializeField] private AnimationCurve stiffnessByHeight = AnimationCurve.Linear(0f, 1.5f, 1f, 0.5f);
    [SerializeField] private float jiggleAmplitude = 1f;
    
    [Header("インタラクション")]
    [SerializeField] private float forceRadius = 0.5f;
    [SerializeField] private float forceStrength = 10f;
    
    [Header("慣性による揺れ")]
    [SerializeField] private float inertiaMultiplier = 2f;
    [SerializeField] private float rotationalInertiaMultiplier = 3f;
    [SerializeField] private float minMovementThreshold = 0.001f;
    
    private Mesh originalMesh;
    private Mesh deformingMesh;
    private Vector3[] originalVertices;
    private Vector3[] displacedVertices;
    private Vector3[] vertexVelocities;
    private float[] vertexMasses;
    private List<int>[] vertexNeighbors;
    
    private float minY, maxY;
    
    // 動き検出用
    private Vector3 previousPosition;
    private Quaternion previousRotation;
    private Vector3 objectVelocity;
    private Vector3 objectAngularVelocity;
    
    void Start()
    {
        InitializeMesh();
        CalculateVertexNeighbors();
        CalculateVertexMasses();
        
        // 初期位置と回転を記録
        previousPosition = transform.position;
        previousRotation = transform.rotation;
    }
    
    void InitializeMesh()
    {
        originalMesh = GetComponent<MeshFilter>().mesh;
        deformingMesh = Instantiate(originalMesh);
        GetComponent<MeshFilter>().mesh = deformingMesh;
        
        originalVertices = originalMesh.vertices;
        displacedVertices = new Vector3[originalVertices.Length];
        vertexVelocities = new Vector3[originalVertices.Length];
        
        System.Array.Copy(originalVertices, displacedVertices, originalVertices.Length);
        
        // Y座標の範囲を計算（高さによる硬さの変化用）
        minY = float.MaxValue;
        maxY = float.MinValue;
        foreach (var v in originalVertices)
        {
            minY = Mathf.Min(minY, v.y);
            maxY = Mathf.Max(maxY, v.y);
        }
    }
    
    void CalculateVertexNeighbors()
    {
        vertexNeighbors = new List<int>[originalVertices.Length];
        for (int i = 0; i < originalVertices.Length; i++)
        {
            vertexNeighbors[i] = new List<int>();
        }
        
        // 三角形を基に隣接頂点を検出
        int[] triangles = originalMesh.triangles;
        for (int i = 0; i < triangles.Length; i += 3)
        {
            int v1 = triangles[i];
            int v2 = triangles[i + 1];
            int v3 = triangles[i + 2];
            
            AddNeighbor(v1, v2);
            AddNeighbor(v2, v3);
            AddNeighbor(v3, v1);
        }
    }
    
    void AddNeighbor(int v1, int v2)
    {
        if (!vertexNeighbors[v1].Contains(v2))
            vertexNeighbors[v1].Add(v2);
        if (!vertexNeighbors[v2].Contains(v1))
            vertexNeighbors[v2].Add(v1);
    }
    
    void CalculateVertexMasses()
    {
        vertexMasses = new float[originalVertices.Length];
        
        // 高さに基づいて質量を調整（上部は軽く、下部は重く）
        for (int i = 0; i < originalVertices.Length; i++)
        {
            float heightRatio = (originalVertices[i].y - minY) / (maxY - minY);
            vertexMasses[i] = mass * (1f + (1f - heightRatio) * 0.5f);
        }
    }
    
    void Update()
    {
        // オブジェクトの動きを検出
        DetectMovement();
        
        // 外部からの力を検出（マウスクリックなど）
        if (Input.GetMouseButton(0))
        {
            Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);
            RaycastHit hit;
            
            if (Physics.Raycast(ray, out hit) && hit.transform == transform)
            {
                ApplyForceAtPoint(hit.point, Vector3.up * forceStrength);
            }
        }
        
        UpdatePhysics();
        UpdateMesh();
        
        // 現在の位置と回転を記録
        previousPosition = transform.position;
        previousRotation = transform.rotation;
    }
    
    void DetectMovement()
    {
        // 位置の変化から速度を計算
        objectVelocity = (transform.position - previousPosition) / Time.deltaTime;
        
        // 回転の変化から角速度を計算
        Quaternion deltaRotation = transform.rotation * Quaternion.Inverse(previousRotation);
        deltaRotation.ToAngleAxis(out float angle, out Vector3 axis);
        objectAngularVelocity = axis * angle * Mathf.Deg2Rad / Time.deltaTime;
        
        // 動きが閾値を超えている場合、慣性力を適用
        if (objectVelocity.magnitude > minMovementThreshold || objectAngularVelocity.magnitude > minMovementThreshold)
        {
            ApplyInertiaForces();
        }
    }
    
    void ApplyInertiaForces()
    {
        for (int i = 0; i < displacedVertices.Length; i++)
        {
            // 高さに基づく影響度（上部ほど影響を受けやすい）
            float heightRatio = (originalVertices[i].y - minY) / (maxY - minY);
            float inertiaEffect = Mathf.Pow(heightRatio, 0.5f); // 上部ほど強い影響
            
            // 並進運動による慣性力（反対方向）
            Vector3 translationalInertia = -objectVelocity * inertiaMultiplier * inertiaEffect;
            
            // 回転運動による慣性力
            Vector3 localPos = originalVertices[i];
            Vector3 rotationalInertia = Vector3.Cross(objectAngularVelocity, localPos) * rotationalInertiaMultiplier * inertiaEffect;
            
            // 慣性力を速度に加算
            vertexVelocities[i] += (translationalInertia + rotationalInertia) / vertexMasses[i];
        }
    }
    
    void UpdatePhysics()
    {
        for (int i = 0; i < displacedVertices.Length; i++)
        {
            Vector3 force = Vector3.zero;
            
            // 重力
            force += Vector3.up * gravity * vertexMasses[i];
            
            // オリジナル位置への復元力（バネ）
            Vector3 displacement = displacedVertices[i] - originalVertices[i];
            float heightRatio = (originalVertices[i].y - minY) / (maxY - minY);
            float localSpringConstant = springConstant * stiffnessByHeight.Evaluate(heightRatio);
            force += -displacement * localSpringConstant;
            
            // 隣接頂点との相互作用（構造保持）
            foreach (int neighborIndex in vertexNeighbors[i])
            {
                Vector3 currentDist = displacedVertices[neighborIndex] - displacedVertices[i];
                Vector3 originalDist = originalVertices[neighborIndex] - originalVertices[i];
                Vector3 springForce = (currentDist.magnitude - originalDist.magnitude) * currentDist.normalized;
                force += springForce * (springConstant * 0.5f);
            }
            
            // 表面張力（スムージング効果）
            if (IsOnSurface(i))
            {
                Vector3 avgPos = Vector3.zero;
                foreach (int neighborIndex in vertexNeighbors[i])
                {
                    avgPos += displacedVertices[neighborIndex];
                }
                if (vertexNeighbors[i].Count > 0)
                {
                    avgPos /= vertexNeighbors[i].Count;
                    force += (avgPos - displacedVertices[i]) * surfaceTension;
                }
            }
            
            // 減衰
            force += -vertexVelocities[i] * damping;
            
            // 速度と位置の更新
            Vector3 acceleration = force / vertexMasses[i];
            vertexVelocities[i] += acceleration * Time.deltaTime;
            
            // 速度制限（安定性のため）
            vertexVelocities[i] = Vector3.ClampMagnitude(vertexVelocities[i], 5f);
            
            // 位置更新
            displacedVertices[i] += vertexVelocities[i] * Time.deltaTime * jiggleAmplitude;
            
            // 底面近くの頂点は動きを制限
            if (heightRatio < 0.1f)
            {
                displacedVertices[i] = Vector3.Lerp(displacedVertices[i], originalVertices[i], 0.5f);
            }
        }
    }
    
    bool IsOnSurface(int vertexIndex)
    {
        // 簡易的な表面判定（より正確にはメッシュの法線情報を使用）
        return vertexNeighbors[vertexIndex].Count < 6;
    }
    
    void ApplyForceAtPoint(Vector3 worldPoint, Vector3 force)
    {
        Vector3 localPoint = transform.InverseTransformPoint(worldPoint);
        
        for (int i = 0; i < displacedVertices.Length; i++)
        {
            float distance = Vector3.Distance(displacedVertices[i], localPoint);
            if (distance < forceRadius)
            {
                float falloff = 1f - (distance / forceRadius);
                vertexVelocities[i] += force * falloff / vertexMasses[i];
            }
        }
    }
    
    void UpdateMesh()
    {
        deformingMesh.vertices = displacedVertices;
        deformingMesh.RecalculateNormals();
        deformingMesh.RecalculateBounds();
    }
    
    // 揺れを起こすパブリックメソッド
    public void Shake(float intensity = 1f)
    {
        for (int i = 0; i < vertexVelocities.Length; i++)
        {
            float heightRatio = (originalVertices[i].y - minY) / (maxY - minY);
            Vector3 randomForce = Random.insideUnitSphere * intensity * heightRatio;
            vertexVelocities[i] += randomForce;
        }
    }
    
    // 特定方向に力を加える
    public void ApplyDirectionalForce(Vector3 direction, float strength)
    {
        for (int i = 0; i < vertexVelocities.Length; i++)
        {
            float heightRatio = (originalVertices[i].y - minY) / (maxY - minY);
            vertexVelocities[i] += direction * strength * heightRatio / vertexMasses[i];
        }
    }
    
    // テスト用：キーボード入力で揺らす
    void TestMovement()
    {
        // 矢印キーで移動テスト
        float moveSpeed = 5f;
        if (Input.GetKey(KeyCode.LeftArrow))
            transform.position += Vector3.left * moveSpeed * Time.deltaTime;
        if (Input.GetKey(KeyCode.RightArrow))
            transform.position += Vector3.right * moveSpeed * Time.deltaTime;
        if (Input.GetKey(KeyCode.UpArrow))
            transform.position += Vector3.forward * moveSpeed * Time.deltaTime;
        if (Input.GetKey(KeyCode.DownArrow))
            transform.position += Vector3.back * moveSpeed * Time.deltaTime;
        
        // QEキーで回転テスト
        if (Input.GetKey(KeyCode.Q))
            transform.Rotate(Vector3.up, -90f * Time.deltaTime);
        if (Input.GetKey(KeyCode.E))
            transform.Rotate(Vector3.up, 90f * Time.deltaTime);
        
        // スペースキーで上下移動
        if (Input.GetKey(KeyCode.Space))
            transform.position += Vector3.up * moveSpeed * Time.deltaTime;
        if (Input.GetKey(KeyCode.LeftShift))
            transform.position += Vector3.down * moveSpeed * Time.deltaTime;
    }
    
    void OnDrawGizmosSelected()
    {
        if (Application.isPlaying && displacedVertices != null)
        {
            Gizmos.color = Color.yellow;
            for (int i = 0; i < displacedVertices.Length; i++)
            {
                if (IsOnSurface(i))
                {
                    Gizmos.DrawWireSphere(transform.TransformPoint(displacedVertices[i]), 0.01f);
                }
            }
        }
    }
}