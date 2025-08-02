using UnityEngine;
using System.Collections.Generic;

/// <summary>
/// プリンのソフトボディ変形を実現するコンポーネント
/// メッシュの頂点を物理シミュレーションで動かし、ぷるぷるとした動きを表現
/// </summary>
[RequireComponent(typeof(MeshFilter))]
[RequireComponent(typeof(MeshRenderer))]
public class PuddingJellyDeformer : MonoBehaviour
{
    [Header("物理パラメータ")]
    [SerializeField] private float springConstant = 50f;        // バネ定数：元の形状に戻る力の強さ
    [SerializeField] private float damping = 5f;                // 減衰係数：揺れが収まる速さ
    [SerializeField] private float mass = 1f;                   // 基準質量：各頂点の重さ
    [SerializeField] private float gravity = -9.81f;            // 重力加速度
    
    [Header("プリン特性")]
    [SerializeField] private float surfaceTension = 2f;         // 表面張力：表面を滑らかに保つ力
    [SerializeField] private AnimationCurve stiffnessByHeight = AnimationCurve.Linear(0f, 1.5f, 1f, 0.5f); // 高さによる硬さの変化
    [SerializeField] private float jiggleAmplitude = 1f;        // 揺れの振幅倍率
    
    [Header("インタラクション")]
    [SerializeField] private float forceRadius = 0.5f;          // 力を加える際の影響半径
    [SerializeField] private float forceStrength = 10f;         // クリック時の力の強さ
    
    [Header("慣性による揺れ")]
    [SerializeField] private float inertiaMultiplier = 2f;      // 移動時の慣性力の倍率
    [SerializeField] private float rotationalInertiaMultiplier = 3f; // 回転時の慣性力の倍率
    [SerializeField] private float minMovementThreshold = 0.001f;    // 揺れを開始する最小移動量
    
    // メッシュ関連の変数
    private Mesh originalMesh;              // 元の形状のメッシュ
    private Mesh deformingMesh;             // 変形用のメッシュ
    private Vector3[] originalVertices;     // 元の頂点位置
    private Vector3[] displacedVertices;    // 変形後の頂点位置
    private Vector3[] vertexVelocities;     // 各頂点の速度
    private float[] vertexMasses;           // 各頂点の質量
    private List<int>[] vertexNeighbors;    // 各頂点の隣接頂点リスト
    
    private float minY, maxY;               // Y座標の最小値と最大値（高さ判定用）
    
    // 動き検出用の変数
    private Vector3 previousPosition;       // 前フレームの位置
    private Quaternion previousRotation;    // 前フレームの回転
    private Vector3 objectVelocity;         // オブジェクトの速度
    private Vector3 objectAngularVelocity;  // オブジェクトの角速度

    /// <summary>
    /// 初期化処理
    /// </summary>
    void Start()
    {
        InitializeMesh();
        CalculateVertexNeighbors();
        CalculateVertexMasses();
        
        // 初期位置と回転を記録
        previousPosition = transform.position;
        previousRotation = transform.rotation;
    }
    
    /// <summary>
    /// メッシュの初期化
    /// オリジナルメッシュをコピーして変形用メッシュを作成
    /// </summary>
    void InitializeMesh()
    {
        // 元のメッシュを取得してコピー
        originalMesh = GetComponent<MeshFilter>().mesh;
        deformingMesh = Instantiate(originalMesh);
        GetComponent<MeshFilter>().mesh = deformingMesh;
        
        // 頂点データの初期化
        originalVertices = originalMesh.vertices;
        displacedVertices = new Vector3[originalVertices.Length];
        vertexVelocities = new Vector3[originalVertices.Length];
        
        // 初期頂点位置をコピー
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
    
    /// <summary>
    /// 各頂点の隣接頂点を計算
    /// メッシュの三角形情報から隣接関係を構築
    /// </summary>
    void CalculateVertexNeighbors()
    {
        // 隣接リストの初期化
        vertexNeighbors = new List<int>[originalVertices.Length];
        for (int i = 0; i < originalVertices.Length; i++)
        {
            vertexNeighbors[i] = new List<int>();
        }
        
        // 三角形を基に隣接頂点を検出
        int[] triangles = originalMesh.triangles;
        for (int i = 0; i < triangles.Length; i += 3)
        {
            // 三角形の3頂点
            int v1 = triangles[i];
            int v2 = triangles[i + 1];
            int v3 = triangles[i + 2];
            
            // 各頂点を隣接関係に追加
            AddNeighbor(v1, v2);
            AddNeighbor(v2, v3);
            AddNeighbor(v3, v1);
        }
    }
    
    /// <summary>
    /// 隣接頂点を追加（重複チェック付き）
    /// </summary>
    void AddNeighbor(int v1, int v2)
    {
        // v1の隣接リストにv2を追加
        if (!vertexNeighbors[v1].Contains(v2))
            vertexNeighbors[v1].Add(v2);
        // v2の隣接リストにv1を追加
        if (!vertexNeighbors[v2].Contains(v1))
            vertexNeighbors[v2].Add(v1);
    }
    
    /// <summary>
    /// 各頂点の質量を計算
    /// 高さに基づいて質量を調整（下部ほど重く）
    /// </summary>
    void CalculateVertexMasses()
    {
        vertexMasses = new float[originalVertices.Length];
        
        // 高さに基づいて質量を調整
        for (int i = 0; i < originalVertices.Length; i++)
        {
            // 高さの比率を計算（0:底面、1:頂上）
            float heightRatio = (originalVertices[i].y - minY) / (maxY - minY);
            // 下部ほど重く（最大1.5倍）
            vertexMasses[i] = mass * (1f + (1f - heightRatio) * 0.5f);
        }
    }
    
    /// <summary>
    /// 毎フレームの更新処理
    /// </summary>
    void Update()
    {
        // オブジェクトの動きを検出
        DetectMovement();
        
        // マウスクリックによる力の適用
        if (Input.GetMouseButton(0))
        {
            Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);
            RaycastHit hit;
            
            // クリック位置がこのオブジェクトの場合
            if (Physics.Raycast(ray, out hit) && hit.transform == transform)
            {
                ApplyForceAtPoint(hit.point, Vector3.up * forceStrength);
            }
        }
        
        // 物理シミュレーションとメッシュの更新
        UpdatePhysics();
        UpdateMesh();
        
        // 現在の位置と回転を記録（次フレーム用）
        previousPosition = transform.position;
        previousRotation = transform.rotation;
    }
    
    /// <summary>
    /// オブジェクトの動きを検出し、慣性力を計算
    /// </summary>
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
    
    /// <summary>
    /// 慣性力を各頂点に適用
    /// オブジェクトの動きと反対方向に力を加えることで揺れを生成
    /// </summary>
    void ApplyInertiaForces()
    {
        for (int i = 0; i < displacedVertices.Length; i++)
        {
            // 高さに基づく影響度（上部ほど影響を受けやすい）
            float heightRatio = (originalVertices[i].y - minY) / (maxY - minY);
            float inertiaEffect = Mathf.Pow(heightRatio, 0.5f); // 平方根で上部を強調
            
            // 並進運動による慣性力（移動と反対方向）
            Vector3 translationalInertia = -objectVelocity * inertiaMultiplier * inertiaEffect;
            
            // 回転運動による慣性力
            Vector3 localPos = originalVertices[i];
            Vector3 rotationalInertia = Vector3.Cross(objectAngularVelocity, localPos) * rotationalInertiaMultiplier * inertiaEffect;
            
            // 慣性力を速度に加算
            vertexVelocities[i] += (translationalInertia + rotationalInertia) / vertexMasses[i];
        }
    }
    
    /// <summary>
    /// 物理シミュレーションの更新
    /// 各頂点に働く力を計算し、位置を更新
    /// </summary>
    void UpdatePhysics()
    {
        for (int i = 0; i < displacedVertices.Length; i++)
        {
            Vector3 force = Vector3.zero;
            
            // 重力の適用
            force += Vector3.up * gravity * vertexMasses[i];
            
            // オリジナル位置への復元力（バネ）
            Vector3 displacement = displacedVertices[i] - originalVertices[i];
            float heightRatio = (originalVertices[i].y - minY) / (maxY - minY);
            // 高さに応じてバネ定数を調整
            float localSpringConstant = springConstant * stiffnessByHeight.Evaluate(heightRatio);
            force += -displacement * localSpringConstant;
            
            // 隣接頂点との相互作用（構造保持）
            foreach (int neighborIndex in vertexNeighbors[i])
            {
                // 現在の距離と元の距離の差分
                Vector3 currentDist = displacedVertices[neighborIndex] - displacedVertices[i];
                Vector3 originalDist = originalVertices[neighborIndex] - originalVertices[i];
                // 距離の変化に基づく復元力
                Vector3 springForce = (currentDist.magnitude - originalDist.magnitude) * currentDist.normalized;
                force += springForce * (springConstant * 0.5f);
            }
            
            // 表面張力（スムージング効果）
            if (IsOnSurface(i))
            {
                // 隣接頂点の平均位置を計算
                Vector3 avgPos = Vector3.zero;
                foreach (int neighborIndex in vertexNeighbors[i])
                {
                    avgPos += displacedVertices[neighborIndex];
                }
                if (vertexNeighbors[i].Count > 0)
                {
                    avgPos /= vertexNeighbors[i].Count;
                    // 平均位置に向かう力（表面を滑らかに）
                    force += (avgPos - displacedVertices[i]) * surfaceTension;
                }
            }
            
            // 速度に比例した減衰力
            force += -vertexVelocities[i] * damping;
            
            // ニュートンの運動方程式（F = ma）から加速度を計算
            Vector3 acceleration = force / vertexMasses[i];
            // 速度の更新（オイラー法）
            vertexVelocities[i] += acceleration * Time.deltaTime;
            
            // 速度制限（数値的安定性のため）
            vertexVelocities[i] = Vector3.ClampMagnitude(vertexVelocities[i], 5f);
            
            // 位置の更新
            displacedVertices[i] += vertexVelocities[i] * Time.deltaTime * jiggleAmplitude;
            
            // 底面近くの頂点は動きを制限（プリンが皿から離れないように）
            if (heightRatio < 0.1f)
            {
                displacedVertices[i] = Vector3.Lerp(displacedVertices[i], originalVertices[i], 0.5f);
            }
        }
    }
    
    /// <summary>
    /// 頂点が表面にあるかどうかを判定
    /// </summary>
    /// <param name="vertexIndex">頂点のインデックス</param>
    /// <returns>表面にある場合はtrue</returns>
    bool IsOnSurface(int vertexIndex)
    {
        // 隣接頂点が6個未満なら表面とみなす（簡易判定）
        return vertexNeighbors[vertexIndex].Count < 6;
    }
    
    /// <summary>
    /// 指定した点に力を加える
    /// </summary>
    /// <param name="worldPoint">力を加える位置（ワールド座標）</param>
    /// <param name="force">加える力のベクトル</param>
    void ApplyForceAtPoint(Vector3 worldPoint, Vector3 force)
    {
        // ワールド座標をローカル座標に変換
        Vector3 localPoint = transform.InverseTransformPoint(worldPoint);
        
        // 影響範囲内の頂点に力を加える
        for (int i = 0; i < displacedVertices.Length; i++)
        {
            float distance = Vector3.Distance(displacedVertices[i], localPoint);
            if (distance < forceRadius)
            {
                // 距離に応じて力を減衰
                float falloff = 1f - (distance / forceRadius);
                vertexVelocities[i] += force * falloff / vertexMasses[i];
            }
        }
    }
    
    /// <summary>
    /// メッシュの頂点データを更新
    /// </summary>
    void UpdateMesh()
    {
        // 変形後の頂点位置を適用
        deformingMesh.vertices = displacedVertices;
        // 法線を再計算（ライティング用）
        deformingMesh.RecalculateNormals();
        // バウンディングボックスを再計算（カリング用）
        deformingMesh.RecalculateBounds();
    }
    
    /// <summary>
    /// プリン全体を揺らす
    /// </summary>
    /// <param name="intensity">揺れの強さ</param>
    public void Shake(float intensity = 1f)
    {
        for (int i = 0; i < vertexVelocities.Length; i++)
        {
            // 高さに応じた強さでランダムな方向に力を加える
            float heightRatio = (originalVertices[i].y - minY) / (maxY - minY);
            Vector3 randomForce = Random.insideUnitSphere * intensity * heightRatio;
            vertexVelocities[i] += randomForce;
        }
    }
    
    /// <summary>
    /// 特定方向に力を加える
    /// </summary>
    /// <param name="direction">力の方向</param>
    /// <param name="strength">力の強さ</param>
    public void ApplyDirectionalForce(Vector3 direction, float strength)
    {
        for (int i = 0; i < vertexVelocities.Length; i++)
        {
            // 高さに応じて力を調整
            float heightRatio = (originalVertices[i].y - minY) / (maxY - minY);
            vertexVelocities[i] += direction * strength * heightRatio / vertexMasses[i];
        }
    }
    
    /// <summary>
    /// キーボード入力による動作テスト
    /// デバッグ用のメソッド
    /// </summary>
    void TestMovement()
    {
        // 移動速度
        float moveSpeed = 5f;
        
        // 矢印キーで水平移動
        if (Input.GetKey(KeyCode.LeftArrow))
            transform.position += Vector3.left * moveSpeed * Time.deltaTime;
        if (Input.GetKey(KeyCode.RightArrow))
            transform.position += Vector3.right * moveSpeed * Time.deltaTime;
        if (Input.GetKey(KeyCode.UpArrow))
            transform.position += Vector3.forward * moveSpeed * Time.deltaTime;
        if (Input.GetKey(KeyCode.DownArrow))
            transform.position += Vector3.back * moveSpeed * Time.deltaTime;
        
        // Q/Eキーで回転
        if (Input.GetKey(KeyCode.Q))
            transform.Rotate(Vector3.up, -90f * Time.deltaTime);
        if (Input.GetKey(KeyCode.E))
            transform.Rotate(Vector3.up, 90f * Time.deltaTime);
        
        // スペース/シフトキーで上下移動
        if (Input.GetKey(KeyCode.Space))
            transform.position += Vector3.up * moveSpeed * Time.deltaTime;
        if (Input.GetKey(KeyCode.LeftShift))
            transform.position += Vector3.down * moveSpeed * Time.deltaTime;
    }
    
    /// <summary>
    /// エディタでのギズモ描画（デバッグ用）
    /// 選択時に表面の頂点を黄色で表示
    /// </summary>
    void OnDrawGizmosSelected()
    {
        // 実行中かつ頂点データが存在する場合
        if (Application.isPlaying && displacedVertices != null)
        {
            Gizmos.color = Color.yellow;
            // 表面の頂点を小さな球で表示
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