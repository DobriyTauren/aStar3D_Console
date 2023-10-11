using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;

public class Node3D
{
    public int X { get; set; }
    public int Y { get; set; }
    public int Z { get; set; }
    public int G { get; set; } // Стоимость движения от начальной точки
    public int H { get; set; } // Эвристическая оценка до конечной точки
    public int F => G + H;     // Сумма G и H
    public Node3D Parent { get; set; } // Родительская точка для восстановления пути

    public Node3D(int x, int y, int z)
    {
        X = x;
        Y = y;
        Z = z;
    }
}

public class Node3DNode
{
    public Node3D Value { get; set; }
    public Node3DNode Left { get; set; }
    public Node3DNode Right { get; set; }

    public Node3DNode(Node3D value)
    {
        Value = value;
    }
}

public class BinarySearchTree
{
    private Node3DNode root;

    public bool Contains(Node3D node)
    {
        return Search(root, node) != null;
    }

    public void Insert(Node3D node)
    {
        root = InsertRec(root, node);
    }

    public Node3D ExtractMin()
    {
        if (root == null)
        {
            throw new InvalidOperationException("The tree is empty.");
        }

        Node3D minNode = FindMin(root);
        root = DeleteRec(root, minNode);
        return minNode;
    }

    public int Count
    {
        get { return CountNodes(root); }
    }

    private Node3DNode Search(Node3DNode root, Node3D node)
    {
        if (root == null || (root.Value.X == node.X && root.Value.Y == node.Y && root.Value.Z == node.Z))
        {
            return root;
        }

        if (root.Value.F > node.F)
        {
            return Search(root.Left, node);
        }
        else
        {
            return Search(root.Right, node);
        }
    }

    private Node3DNode InsertRec(Node3DNode root, Node3D node)
    {
        if (root == null)
        {
            return new Node3DNode(node);
        }

        if (node.F < root.Value.F)
        {
            root.Left = InsertRec(root.Left, node);
        }
        else
        {
            root.Right = InsertRec(root.Right, node);
        }

        return root;
    }

    private Node3D FindMin(Node3DNode root)
    {
        while (root.Left != null)
        {
            root = root.Left;
        }

        return root.Value;
    }

    private Node3DNode DeleteRec(Node3DNode root, Node3D node)
    {
        if (root == null)
        {
            return root;
        }

        if (node.F < root.Value.F)
        {
            root.Left = DeleteRec(root.Left, node);
        }
        else if (node.F > root.Value.F)
        {
            root.Right = DeleteRec(root.Right, node);
        }
        else
        {
            if (root.Left == null)
            {
                return root.Right;
            }
            else if (root.Right == null)
            {
                return root.Left;
            }

            root.Value = FindMin(root.Right);
            root.Right = DeleteRec(root.Right, root.Value);
        }

        return root;
    }

    private int CountNodes(Node3DNode root)
    {
        if (root == null)
        {
            return 0;
        }

        return 1 + CountNodes(root.Left) + CountNodes(root.Right);
    }
}

public class AStar3D_Heap
{
    private const int LIMIT_MULTIPLAYER = 2;

    public static List<Node3D> FindPath(int[,,] grid, Node3D start, Node3D end)
    {
        int width = grid.GetLength(0);
        int height = grid.GetLength(1);
        int depth = grid.GetLength(2);

        // Генерация соседей
        int[] dx = { -1, 1, 0, 0, 0, 0 };
        int[] dy = { 0, 0, -1, 1, 0, 0 };
        int[] dz = { 0, 0, 0, 0, -1, 1 };

        int nodesCount = width * height * depth;

        // Инициализируем кучу для открытого списка
        MinHeap openList = new MinHeap();
        openList.Insert(start);

        List<Node3D> closedList = new List<Node3D>();

        while (openList.Count > 0 && !IsLooping(openList.Count, closedList.Count, nodesCount))
        {
            Node3D current = openList.ExtractMin();
            closedList.Add(current);

            if (current.X == end.X && current.Y == end.Y && current.Z == end.Z)
            {
                Console.WriteLine($"total nodes: {openList.Count + closedList.Count}");
                // Путь найден, восстанавливаем его
                return ReconstructPath(current);
            }

            for (int i = 0; i < 6; i++)
            {
                int newX = current.X + dx[i];
                int newY = current.Y + dy[i];
                int newZ = current.Z + dz[i];

                if (IsInsideGrid(newX, newY, newZ, width, height, depth) && grid[newX, newY, newZ] == 0)
                {
                    Node3D neighbor = new Node3D(newX, newY, newZ);

                    if (closedList.Contains(neighbor))
                    {
                        continue;
                    }

                    int tentativeG = current.G + 1;

                    if (!openList.Contains(neighbor) || tentativeG < neighbor.G)
                    {
                        neighbor.Parent = current;
                        neighbor.G = tentativeG;
                        neighbor.H = Math.Abs(newX - end.X) + Math.Abs(newY - end.Y) + Math.Abs(newZ - end.Z);

                        if (!openList.Contains(neighbor))
                        {
                            openList.Insert(neighbor);
                        }
                    }
                }
            }
        }

        Console.WriteLine($"total nodes: {openList.Count + closedList.Count}");
        // Путь не найден
        return null;
    }

    private static bool IsLooping(int openListCount, int closedListCount, int nodesCount)
    {
        if (openListCount + closedListCount > nodesCount * LIMIT_MULTIPLAYER)
        {
            return true;
        }

        return false;
    }

    private static List<Node3D> ReconstructPath(Node3D node)
    {
        List<Node3D> path = new List<Node3D>();

        while (node != null)
        {
            path.Add(node);
            node = node.Parent;
        }

        path.Reverse();
        return path;
    }

    private static bool IsInsideGrid(int x, int y, int z, int width, int height, int depth)
    {
        return x >= 0 && x < width && y >= 0 && y < height && z >= 0 && z < depth;
    }


    public class MinHeap
    {
        private List<Node3D> heap = new List<Node3D>();

        public int Count => heap.Count;

        public void Insert(Node3D node)
        {
            heap.Add(node);
            HeapifyUp(heap.Count - 1);
        }

        public Node3D ExtractMin()
        {
            if (heap.Count == 0)
                throw new InvalidOperationException("The heap is empty.");

            Node3D min = heap[0];
            int lastIndex = heap.Count - 1;
            heap[0] = heap[lastIndex];
            heap.RemoveAt(lastIndex);

            if (heap.Count > 0)
            {
                HeapifyDown(0);
            }

            return min;
        }

        public bool Contains(Node3D node)
        {
            return heap.Contains(node);
        }

        private void HeapifyUp(int index)
        {
            int parentIndex = (index - 1) / 2;
            while (index > 0 && Compare(heap[index], heap[parentIndex]) < 0)
            {
                Swap(index, parentIndex);
                index = parentIndex;
                parentIndex = (index - 1) / 2;
            }
        }

        private void HeapifyDown(int index)
        {
            int leftChildIndex = 2 * index + 1;
            int rightChildIndex = 2 * index + 2;
            int smallest = index;

            if (leftChildIndex < heap.Count && Compare(heap[leftChildIndex], heap[index]) < 0)
            {
                smallest = leftChildIndex;
            }

            if (rightChildIndex < heap.Count && Compare(heap[rightChildIndex], heap[smallest]) < 0)
            {
                smallest = rightChildIndex;
            }

            if (smallest != index)
            {
                Swap(index, smallest);
                HeapifyDown(smallest);
            }
        }

        private void Swap(int i, int j)
        {
            Node3D temp = heap[i];
            heap[i] = heap[j];
            heap[j] = temp;
        }

        private int Compare(Node3D a, Node3D b)
        {
            int f1 = a.G + a.H;
            int f2 = b.G + b.H;
            return f1.CompareTo(f2);
        }
    }
}

public class AStar3D_BinaryTree
{
    private const int LIMIT_MULTIPLAYER = 2;

    public static List<Node3D> FindPath(int[,,] grid, Node3D start, Node3D end)
    {
        int width = grid.GetLength(0);
        int height = grid.GetLength(1);
        int depth = grid.GetLength(2);

        // Генерация соседей
        int[] dx = { -1, 1, 0, 0, 0, 0 };
        int[] dy = { 0, 0, -1, 1, 0, 0 };
        int[] dz = { 0, 0, 0, 0, -1, 1 };

        int nodesCount = height * width * depth;

        // Инициализируем бинарное дерево
        BinarySearchTree openList = new BinarySearchTree();
        openList.Insert(start);

        List<Node3D> closedList = new List<Node3D>();
        object lockObject = new object(); // Объект для блокировки доступа к общим данным

        while (openList.Count > 0 && !IsLooping(openList.Count, closedList.Count, nodesCount))
        {
            Node3D current = openList.ExtractMin();

            lock (lockObject)
            {
                closedList.Add(current);
            }

            if (current.X == end.X && current.Y == end.Y && current.Z == end.Z)
            {
                Console.WriteLine($"total nodes: {openList.Count + closedList.Count}");

                // Путь найден, восстанавливаем его
                return ReconstructPath(current);
            }

            Parallel.For(0, 6, i =>
            {
                int newX = current.X + dx[i];
                int newY = current.Y + dy[i];
                int newZ = current.Z + dz[i];

                if (IsInsideGrid(newX, newY, newZ, width, height, depth) && grid[newX, newY, newZ] == 0)
                {
                    Node3D neighbor = new Node3D(newX, newY, newZ);

                    bool inClosedList;
                    lock (lockObject)
                    {
                        inClosedList = closedList.Any(c => c.X == neighbor.X && c.Y == neighbor.Y && c.Z == neighbor.Z);
                    }

                    if (!inClosedList)
                    {
                        int tentativeG = current.G + 1;

                        bool inOpenList;
                        lock (lockObject)
                        {
                            inOpenList = openList.Contains(neighbor);
                        }

                        if (!inOpenList || tentativeG < neighbor.G)
                        {
                            neighbor.Parent = current;
                            neighbor.G = tentativeG;
                            neighbor.H = Math.Abs(newX - end.X) + Math.Abs(newY - end.Y) + Math.Abs(newZ - end.Z);

                            lock (lockObject)
                            {
                                if (!inOpenList)
                                {
                                    openList.Insert(neighbor);
                                }
                            }
                        }
                    }
                }
            });
        }

        // Путь не найден
        Console.WriteLine($"total nodes: {openList.Count + closedList.Count}");

        return null;
    }

    private static List<Node3D> ReconstructPath(Node3D node)
    {
        List<Node3D> path = new List<Node3D>();

        while (node != null)
        {
            path.Add(node);
            node = node.Parent;
        }

        path.Reverse();
        return path;
    }

    private static bool IsInsideGrid(int x, int y, int z, int width, int height, int depth)
    {
        return x >= 0 && x < width && y >= 0 && y < height && z >= 0 && z < depth;
    }

    private static bool IsLooping(int openListCount, int closedListCount, int nodesCount)
    {
        if (openListCount + closedListCount > nodesCount * LIMIT_MULTIPLAYER)
        {
            return true;
        }

        return false;
    }
}

public class AStar3D
{
    private const int LIMIT_MULTIPLAYER = 2;

    public static List<Node3D> FindPathOneThread(int[,,] grid, Node3D start, Node3D end)
    {
        int width = grid.GetLength(0);
        int height = grid.GetLength(1);
        int depth = grid.GetLength(2);

        int nodesCount = width * height * depth;

        List<Node3D> openList = new List<Node3D>();
        List<Node3D> closedList = new List<Node3D>();

        // Генерация соседей
        int[] dx = { -1, 1, 0, 0, 0, 0 };
        int[] dy = { 0, 0, -1, 1, 0, 0 };
        int[] dz = { 0, 0, 0, 0, -1, 1 };

        openList.Add(start);

        while (openList.Count > 0 && !IsLooping(openList.Count, closedList.Count, nodesCount))
        {
            Node3D current = openList[0];

            // Находим узел с наименьшей оценкой F
            for (int i = 1; i < openList.Count; i++)
            {
                if (openList[i].F < current.F)
                {
                    current = openList[i];
                }
            }

            openList.Remove(current);
            closedList.Add(current);

            if (current.X == end.X && current.Y == end.Y && current.Z == end.Z)
            {
                // Путь найден, восстанавливаем его
                Console.WriteLine($"total nodes: {openList.Count + closedList.Count}");

                return ReconstructPath(current);
            }


            for (int i = 0; i < 6; i++)
            {
                int newX = current.X + dx[i];
                int newY = current.Y + dy[i];
                int newZ = current.Z + dz[i];

                if (IsInsideGrid(newX, newY, newZ, width, height, depth) && grid[newX, newY, newZ] == 0)
                {
                    Node3D neighbor = new Node3D(newX, newY, newZ);

                    if (closedList.Contains(neighbor))
                    {
                        continue;
                    }

                    int tentativeG = current.G + 1;

                    if (!openList.Contains(neighbor) || tentativeG < neighbor.G)
                    {
                        neighbor.Parent = current;
                        neighbor.G = tentativeG;
                        neighbor.H = Math.Abs(newX - end.X) + Math.Abs(newY - end.Y) + Math.Abs(newZ - end.Z);

                        if (!openList.Contains(neighbor))
                        {
                            openList.Add(neighbor);
                        }
                    }
                }
            }
        }
        Console.WriteLine($"total nodes: {openList.Count + closedList.Count}");

        // Путь не найден
        return null;
    }

    public static List<Node3D> FindPathMultiThread_ParallelFor(int[,,] grid, Node3D start, Node3D end)
    {
        int width = grid.GetLength(0);
        int height = grid.GetLength(1);
        int depth = grid.GetLength(2);

        List<Node3D> openList = new List<Node3D>();
        List<Node3D> closedList = new List<Node3D>();
        object lockObject = new object(); // Объект для блокировки доступа к общим данным

        openList.Add(start);

        while (openList.Count > 0)
        {
            Node3D current = openList[0];

            // Находим узел с наименьшей оценкой F
            for (int i = 1; i < openList.Count; i++)
            {
                if (openList[i].F < current.F)
                {
                    current = openList[i];
                }
            }

            openList.Remove(current);
            closedList.Add(current);

            if (current.X == end.X && current.Y == end.Y && current.Z == end.Z)
            {
                // Путь найден, восстанавливаем его
                return ReconstructPath(current);
            }

            // Генерация соседей
            int[] dx = { -1, 1, 0, 0, 0, 0 };
            int[] dy = { 0, 0, -1, 1, 0, 0 };
            int[] dz = { 0, 0, 0, 0, -1, 1 };

            // Используем параллельную обработку соседей
            Parallel.For(0, 6, i =>
            {
                int newX = current.X + dx[i];
                int newY = current.Y + dy[i];
                int newZ = current.Z + dz[i];

                if (IsInsideGrid(newX, newY, newZ, width, height, depth) && grid[newX, newY, newZ] == 0)
                {
                    Node3D neighbor = new Node3D(newX, newY, newZ);

                    lock (lockObject) // Блокируем доступ к общим данным
                    {
                        if (closedList.Contains(neighbor))
                        {
                            return;
                        }

                        int tentativeG = current.G + 1;

                        if (!openList.Contains(neighbor) || tentativeG < neighbor.G)
                        {
                            neighbor.Parent = current;
                            neighbor.G = tentativeG;
                            neighbor.H = Math.Abs(newX - end.X) + Math.Abs(newY - end.Y) + Math.Abs(newZ - end.Z);

                            if (!openList.Contains(neighbor))
                            {
                                openList.Add(neighbor);
                            }
                        }
                    }
                }
            });
        }

        // Путь не найден
        return null;
    }

    public static List<Node3D> FindPathMultiThread_ParallelForEach(int[,,] grid, Node3D start, Node3D end)
    {
        int width = grid.GetLength(0);
        int height = grid.GetLength(1);
        int depth = grid.GetLength(2);

        List<Node3D> openList = new List<Node3D>();
        List<Node3D> closedList = new List<Node3D>();
        object lockObject = new object(); // Объект для блокировки доступа к общим данным

        openList.Add(start);

        while (openList.Count > 0)
        {
            Node3D current = openList[0];

            // Находим узел с наименьшей оценкой F
            for (int i = 1; i < openList.Count; i++)
            {
                if (openList[i].F < current.F)
                {
                    current = openList[i];
                }
            }

            openList.Remove(current);
            closedList.Add(current);

            if (current.X == end.X && current.Y == end.Y && current.Z == end.Z)
            {
                // Путь найден, восстанавливаем его
                return ReconstructPath(current);
            }

            // Генерация соседей с использованием параллельной обработки
            Parallel.ForEach(GetNeighborDirections(), direction =>
            {
                int newX = current.X + direction[0];
                int newY = current.Y + direction[1];
                int newZ = current.Z + direction[2];

                if (IsInsideGrid(newX, newY, newZ, width, height, depth) && grid[newX, newY, newZ] == 0)
                {
                    Node3D neighbor = new Node3D(newX, newY, newZ);

                    lock (lockObject) // Блокируем доступ к общим данным
                    {
                        if (closedList.Contains(neighbor))
                        {
                            return;
                        }

                        int tentativeG = current.G + 1;

                        if (!openList.Contains(neighbor) || tentativeG < neighbor.G)
                        {
                            neighbor.Parent = current;
                            neighbor.G = tentativeG;
                            neighbor.H = Math.Abs(newX - end.X) + Math.Abs(newY - end.Y) + Math.Abs(newZ - end.Z);

                            if (!openList.Contains(neighbor))
                            {
                                openList.Add(neighbor);
                            }
                        }
                    }
                }
            });
        }

        // Путь не найден
        return null;
    }

    public static List<Node3D> FindPath_PLINQ(int[,,] grid, Node3D start, Node3D end)
    {
        int width = grid.GetLength(0);
        int height = grid.GetLength(1);
        int depth = grid.GetLength(2);

        int nodesCount = width * height * depth;

        List<Node3D> openList = new List<Node3D>();
        List<Node3D> closedList = new List<Node3D>();
        object lockObject = new object(); // Объект для блокировки доступа к общим данным

        openList.Add(start);

        while (openList.Count > 0 && !IsLooping(openList.Count, closedList.Count, nodesCount))
        {
            Node3D current = openList[0];

            // Находим узел с наименьшей оценкой F
            for (int i = 1; i < openList.Count; i++)
            {
                if (openList[i].F < current.F)
                {
                    current = openList[i];
                }
            }

            openList.Remove(current);
            closedList.Add(current);

            if (current.X == end.X && current.Y == end.Y && current.Z == end.Z)
            {
                // Путь найден, восстанавливаем его
                return ReconstructPath(current);
            }

            // Генерация соседей с использованием PLINQ
            var neighborDirections = GetNeighborDirections();
            var parallelQuery = neighborDirections.AsParallel().WithDegreeOfParallelism(Environment.ProcessorCount);

            parallelQuery.ForAll(direction =>
            {
                int newX = current.X + direction[0];
                int newY = current.Y + direction[1];
                int newZ = current.Z + direction[2];

                if (IsInsideGrid(newX, newY, newZ, width, height, depth) && grid[newX, newY, newZ] == 0)
                {
                    Node3D neighbor = new Node3D(newX, newY, newZ);

                    lock (lockObject) // Блокируем доступ к общим данным
                    {
                        if (closedList.Contains(neighbor))
                        {
                            return;
                        }

                        int tentativeG = current.G + 1;

                        if (!openList.Contains(neighbor) || tentativeG < neighbor.G)
                        {
                            neighbor.Parent = current;
                            neighbor.G = tentativeG;
                            neighbor.H = Math.Abs(newX - end.X) + Math.Abs(newY - end.Y) + Math.Abs(newZ - end.Z);

                            if (!openList.Contains(neighbor))
                            {
                                openList.Add(neighbor);
                            }
                        }
                    }
                }
            });
        }

        // Путь не найден
        return null;
    }

    public static List<Node3D> FindPath_TPL(int[,,] grid, Node3D start, Node3D end)
    {
        int width = grid.GetLength(0);
        int height = grid.GetLength(1);
        int depth = grid.GetLength(2);

        List<Node3D> openList = new List<Node3D>();
        List<Node3D> closedList = new List<Node3D>();
        object lockObject = new object(); // Объект для блокировки доступа к общим данным

        openList.Add(start);

        while (openList.Count > 0)
        {
            Node3D current = openList[0];

            // Находим узел с наименьшей оценкой F
            for (int i = 1; i < openList.Count; i++)
            {
                if (openList[i].F < current.F)
                {
                    current = openList[i];
                }
            }

            openList.Remove(current);
            closedList.Add(current);

            if (current.X == end.X && current.Y == end.Y && current.Z == end.Z)
            {
                // Путь найден, восстанавливаем его
                return ReconstructPath(current);
            }

            // Генерация соседей с использованием асинхронных задач
            var neighborDirections = GetNeighborDirections();

            // Используем ConcurrentBag для сбора результатов асинхронных задач
            ConcurrentBag<Node3D> newNodes = new ConcurrentBag<Node3D>();

            Parallel.ForEach(neighborDirections, direction =>
            {
                int newX = current.X + direction[0];
                int newY = current.Y + direction[1];
                int newZ = current.Z + direction[2];

                if (IsInsideGrid(newX, newY, newZ, width, height, depth) && grid[newX, newY, newZ] == 0)
                {
                    Node3D neighbor = new Node3D(newX, newY, newZ);

                    lock (lockObject) // Блокируем доступ к общим данным
                    {
                        if (closedList.Contains(neighbor))
                        {
                            return;
                        }

                        int tentativeG = current.G + 1;

                        if (!openList.Contains(neighbor) || tentativeG < neighbor.G)
                        {
                            neighbor.Parent = current;
                            neighbor.G = tentativeG;
                            neighbor.H = Math.Abs(newX - end.X) + Math.Abs(newY - end.Y) + Math.Abs(newZ - end.Z);

                            if (!openList.Contains(neighbor))
                            {
                                newNodes.Add(neighbor);
                            }
                        }
                    }
                }
            });

            // Добавляем новые узлы в openList
            foreach (var newNode in newNodes)
            {
                openList.Add(newNode);
            }
        }

        // Путь не найден
        return null;
    }

    public static List<Node3D> FindPath_ThreadPool(int[,,] grid, Node3D start, Node3D end)
    {
        int width = grid.GetLength(0);
        int height = grid.GetLength(1);
        int depth = grid.GetLength(2);

        List<Node3D> openList = new List<Node3D>();
        List<Node3D> closedList = new List<Node3D>();
        object lockObject = new object(); // Объект для блокировки доступа к общим данным

        openList.Add(start);

        while (openList.Count > 0)
        {
            Node3D current = openList[0];

            // Находим узел с наименьшей оценкой F
            for (int i = 1; i < openList.Count; i++)
            {
                if (openList[i].F < current.F)
                {
                    current = openList[i];
                }
            }

            openList.Remove(current);
            closedList.Add(current);

            if (current.X == end.X && current.Y == end.Y && current.Z == end.Z)
            {
                // Путь найден, восстанавливаем его
                return ReconstructPath(current);
            }

            // Генерация соседей
            int[] dx = { -1, 1, 0, 0, 0, 0 };
            int[] dy = { 0, 0, -1, 1, 0, 0 };
            int[] dz = { 0, 0, 0, 0, -1, 1 };

            List<Node3D> neighbors = new List<Node3D>();

            for (int i = 0; i < 6; i++)
            {
                int newX = current.X + dx[i];
                int newY = current.Y + dy[i];
                int newZ = current.Z + dz[i];

                if (IsInsideGrid(newX, newY, newZ, width, height, depth) && grid[newX, newY, newZ] == 0)
                {
                    Node3D neighbor = new Node3D(newX, newY, newZ);
                    neighbors.Add(neighbor);
                }
            }

            // Параллельно обрабатываем соседей
            Parallel.ForEach(neighbors, neighbor =>
            {
                lock (lockObject)
                {
                    if (closedList.Contains(neighbor))
                    {
                        return;
                    }

                    int tentativeG = current.G + 1;

                    if (!openList.Contains(neighbor) || tentativeG < neighbor.G)
                    {
                        neighbor.Parent = current;
                        neighbor.G = tentativeG;
                        neighbor.H = Math.Abs(neighbor.X - end.X) + Math.Abs(neighbor.Y - end.Y) + Math.Abs(neighbor.Z - end.Z);

                        if (!openList.Contains(neighbor))
                        {
                            openList.Add(neighbor);
                        }
                    }
                }
            });
        }

        // Путь не найден
        return null;
    }

    public static List<Node3D> FindPath_AsyncTasks(int[,,] grid, Node3D start, Node3D end)
    {
        int width = grid.GetLength(0);
        int height = grid.GetLength(1);
        int depth = grid.GetLength(2);

        List<Node3D> openList = new List<Node3D>();
        List<Node3D> closedList = new List<Node3D>();
        object lockObject = new object(); // Объект для блокировки доступа к общим данным

        openList.Add(start);

        while (openList.Count > 0)
        {
            Node3D current = openList[0];

            // Находим узел с наименьшей оценкой F
            for (int i = 1; i < openList.Count; i++)
            {
                if (openList[i].F < current.F)
                {
                    current = openList[i];
                }
            }

            openList.Remove(current);
            closedList.Add(current);

            if (current.X == end.X && current.Y == end.Y && current.Z == end.Z)
            {
                // Путь найден, восстанавливаем его
                return ReconstructPath(current);
            }

            // Генерация соседей
            int[] dx = { -1, 1, 0, 0, 0, 0 };
            int[] dy = { 0, 0, -1, 1, 0, 0 };
            int[] dz = { 0, 0, 0, 0, -1, 1 };

            List<Task> neighborTasks = new List<Task>();

            for (int i = 0; i < 6; i++)
            {
                int newX = current.X + dx[i];
                int newY = current.Y + dy[i];
                int newZ = current.Z + dz[i];

                if (IsInsideGrid(newX, newY, newZ, width, height, depth) && grid[newX, newY, newZ] == 0)
                {
                    Node3D neighbor = new Node3D(newX, newY, newZ);
                    neighborTasks.Add(ProcessNeighborAsync(neighbor, current, end, openList, closedList, lockObject));
                }
            }

            Task.WhenAll(neighborTasks).Wait(); // Ждем завершения всех задач соседей
        }

        // Путь не найден
        return null;
    }

    private static async Task ProcessNeighborAsync(Node3D neighbor, Node3D current, Node3D end, List<Node3D> openList, List<Node3D> closedList, object lockObject)
    {
        lock (lockObject)
        {
            if (closedList.Contains(neighbor))
            {
                return;
            }

            int tentativeG = current.G + 1;

            if (!openList.Contains(neighbor) || tentativeG < neighbor.G)
            {
                neighbor.Parent = current;
                neighbor.G = tentativeG;
                neighbor.H = Math.Abs(neighbor.X - end.X) + Math.Abs(neighbor.Y - end.Y) + Math.Abs(neighbor.Z - end.Z);

                if (!openList.Contains(neighbor))
                {
                    openList.Add(neighbor);
                }
            }
        }
    }

    private static List<int[]> GetNeighborDirections()
    {
        // Возвращает массив направлений движения [dx, dy, dz]
        return new List<int[]>
    {
        new int[] { -1, 0, 0 },
        new int[] { 1, 0, 0 },
        new int[] { 0, -1, 0 },
        new int[] { 0, 1, 0 },
        new int[] { 0, 0, -1 },
        new int[] { 0, 0, 1 }
    };
    }

    private static List<Node3D> ReconstructPath(Node3D node)
    {
        List<Node3D> path = new List<Node3D>();

        while (node != null)
        {
            path.Add(node);
            node = node.Parent;
        }

        path.Reverse();
        return path;
    }

    private static bool IsInsideGrid(int x, int y, int z, int width, int height, int depth)
    {
        return x >= 0 && x < width && y >= 0 && y < height && z >= 0 && z < depth;
    }

    private static bool IsLooping (int openListCount, int closedListCount, int nodesCount)
    {
        if (openListCount + closedListCount > nodesCount * LIMIT_MULTIPLAYER)
        {
            return true;
        }

        return false;
    }
}

class Program
{
    private static DateTime _startTime = DateTime.Now;
    static void Main(string[] args)
    {
        Node3D start = new Node3D(0, 0, 0);
        Node3D end = new Node3D(11, 11, 11);
        
        FindPathAlgorithm[] findPathAlgorithms = new FindPathAlgorithm[] 
        { 
            AStar3D_Heap.FindPath,
            AStar3D_BinaryTree.FindPath,
            AStar3D.FindPathOneThread, 
            AStar3D.FindPath_PLINQ,
            
            //AStar3D.FindPathMultiThread_ParallelFor, AStar3D.FindPathMultiThread_ParallelForEach,
            //AStar3D.FindPath_TPL, AStar3D.FindPath_ThreadPool, AStar3D.FindPath_AsyncTasks,

        };


        do
        {
            _startTime = DateTime.Now;

            int[,,] grid = GenerateRandomGrid(16, 16, 16);

            grid[end.X, end.Y, end.Z] = 0;
            grid[start.X, start.Y, start.Z] = 0;

            CheckAlgorithmsSpeed(grid, start, end, findPathAlgorithms);

            Console.WriteLine($"------------------------------------\ndone! time spend: {(DateTime.Now - _startTime).TotalSeconds}(s)\n");
            
        }
        while (Console.ReadKey().Key == ConsoleKey.Enter);
    }

    private static void CheckAlgorithmsSpeed(int[,,] grid, Node3D start, Node3D end, params FindPathAlgorithm[] algorithms)
    {
        double prevSeconds = 0;
        List<Node3D> path = new List<Node3D>();

        for (int i = 0; i < algorithms.Length; i++)
        {
            path = algorithms[i](grid, start, end);

            if (path != null)
            {
                //foreach (Node3D node in path)
                //{
                //    Console.WriteLine($"({node.X}, {node.Y}, {node.Z})");
                //}

                double totalSeconds = (DateTime.Now - _startTime).TotalSeconds;
                Console.WriteLine($"[{i}] path found! time spend: {totalSeconds - prevSeconds}(s)");

                prevSeconds = totalSeconds;
            }
            else
            {
                Console.WriteLine($"[{i}] path not found.");
                break;
            }
        }
    }

    public delegate List<Node3D> FindPathAlgorithm(int[,,] grid, Node3D start, Node3D end);

    static int[,,] GenerateRandomGrid(int xSize, int ySize, int zSize)
    {
        Random random = new Random();
        int[,,] grid = new int[xSize, ySize, zSize];

        for (int x = 0; x < xSize; x++)
        {
            for (int y = 0; y < ySize; y++)
            {
                for (int z = 0; z < zSize; z++)
                {
                    // Генерируем случайное число от 0 до 1
                    int randomNumber = random.Next(2);
                    grid[x, y, z] = randomNumber;
                }
            }
        }

        return grid;
    }
}
