using UnityEngine;
using System.Collections.Generic;
using System.Linq; // Diperlukan untuk OrderBy (jika pakai List) atau PriorityQueue

public static class AStarPathfinder
{
    // Helper class/struct untuk menyimpan data node A*
    private class Node : System.IComparable<Node>
    {
        public Vector2Int Position;
        public float GCost; // Cost from start
        public float HCost; // Heuristic cost to end
        public float FCost => GCost + HCost; // Total estimated cost
        public Node Parent;

        public Node(Vector2Int pos) { Position = pos; }

        // Perbandingan untuk Priority Queue (prioritaskan F cost terendah)
        public int CompareTo(Node other)
        {
             int compare = FCost.CompareTo(other.FCost);
             if (compare == 0) // Jika F cost sama, prioritaskan H cost terendah
             {
                 compare = HCost.CompareTo(other.HCost);
             }
             return compare;
        }

         // Override Equals dan GetHashCode jika menggunakan HashSet
        public override bool Equals(object obj) => obj is Node other && Position.Equals(other.Position);
        public override int GetHashCode() => Position.GetHashCode();
    }

    /// <summary>
    /// Mencari jalur terpendek dari start ke target menggunakan A*.
    /// </summary>
    /// <param name="startPos">Posisi grid awal.</param>
    /// <param name="targetPos">Posisi grid tujuan.</param>
    /// <param name="map">Referensi ke GridMappingSystem.</param>
    /// <returns>List Vector2Int berisi jalur (termasuk start, tidak termasuk target jika tidak tercapai), atau null jika tidak ada jalur.</returns>
    public static List<Vector2Int> FindPath(Vector2Int startPos, Vector2Int targetPos, GridMappingSystem map)
    {
        // --- Setup A* ---
        Node startNode = new Node(startPos);
        Node targetNode = new Node(targetPos);

        // Ganti List dengan implementasi PriorityQueue jika performa jadi masalah
        List<Node> openSet = new List<Node>();
        HashSet<Vector2Int> closedSet = new HashSet<Vector2Int>(); // Lacak posisi yang sudah dievaluasi

        openSet.Add(startNode);

        // Dictionary untuk menyimpan GCost dan Parent ter-efisien sejauh ini
        Dictionary<Vector2Int, float> gCostMap = new Dictionary<Vector2Int, float>();
        Dictionary<Vector2Int, Node> cameFrom = new Dictionary<Vector2Int, Node>();

        gCostMap[startPos] = 0;
        startNode.GCost = 0;
        startNode.HCost = CalculateHeuristic(startPos, targetPos);


        // --- A* Loop ---
        while (openSet.Count > 0)
        {
            // Dapatkan node dengan F Cost terendah (simulasi PriorityQueue dengan List.Sort)
            openSet.Sort(); // Urutkan berdasarkan CompareTo di Node class
            Node currentNode = openSet[0];
            openSet.RemoveAt(0);

            // Jika sudah sampai target
            if (currentNode.Position == targetPos)
            {
                return ReconstructPath(currentNode, cameFrom);
            }

            closedSet.Add(currentNode.Position); // Tandai sudah dievaluasi

            // --- Cek Tetangga ---
            foreach (Vector2Int neighborPos in map.GetNeighbors(currentNode.Position, true)) // Gunakan includeDiagonals=true
            {
                // Lewati jika sudah dievaluasi
                if (closedSet.Contains(neighborPos)) continue;

                // Hitung G Cost baru ke tetangga
                float distanceToNeighbor = Vector2Int.Distance(currentNode.Position, neighborPos); // Akan ~1 atau ~1.414
                float tentativeGCost = currentNode.GCost + distanceToNeighbor;

                // Jika jalur baru ke tetangga ini lebih baik ATAU tetangga belum pernah dikunjungi
                 bool neighborInOpenSet = openSet.Any(n => n.Position == neighborPos);
                 float currentNeighborGCost = gCostMap.ContainsKey(neighborPos) ? gCostMap[neighborPos] : float.MaxValue;

                if (tentativeGCost < currentNeighborGCost)
                {
                    // Update jalur terbaik ke tetangga ini
                    cameFrom[neighborPos] = currentNode;
                    gCostMap[neighborPos] = tentativeGCost;

                    Node neighborNode = openSet.FirstOrDefault(n => n.Position == neighborPos);
                    if (neighborNode == null) {
                         neighborNode = new Node(neighborPos);
                         // Hitung H Cost hanya sekali saat node baru dibuat
                         neighborNode.HCost = CalculateHeuristic(neighborPos, targetPos);
                         openSet.Add(neighborNode); // Tambahkan ke openSet jika baru
                    }
                    // Update GCost (dan F Cost otomatis terupdate)
                    neighborNode.GCost = tentativeGCost;

                }
            }
        }

        // Tidak ada jalur ditemukan
        Debug.LogWarning($"A* Path not found from {startPos} to {targetPos}");
        return null;
    }

    /// <summary>
    /// Membangun ulang jalur dari node akhir ke awal.
    /// </summary>
    private static List<Vector2Int> ReconstructPath(Node endNode, Dictionary<Vector2Int, Node> cameFrom)
    {
        List<Vector2Int> path = new List<Vector2Int>();
        Node current = endNode;
        while (current != null)
        {
            path.Add(current.Position);
            // Mundur ke parent, pastikan key ada sebelum akses
             current = cameFrom.ContainsKey(current.Position) ? cameFrom[current.Position] : null;
            // Pengaman loop tak terbatas jika ada masalah di cameFrom
             if(path.Count > cameFrom.Count + 5) { // Heuristik batas
                 Debug.LogError("A* Path reconstruction seems stuck!");
                 return path; // Kembalikan path parsial
             }
        }
        path.Reverse(); // Balik urutan agar dari start ke end
        return path;
    }

    /// <summary>
    /// Menghitung estimasi biaya (Heuristic) dari A ke B (jarak Euclidean).
    /// </summary>
    private static float CalculateHeuristic(Vector2Int a, Vector2Int b)
    {
        return Vector2Int.Distance(a, b); // Jarak lurus (Euclidean)
        // Alternatif: Manhattan Distance (lebih cepat, tapi kurang akurat untuk diagonal)
        // return Mathf.Abs(a.x - b.x) + Mathf.Abs(a.y - b.y);
    }
}