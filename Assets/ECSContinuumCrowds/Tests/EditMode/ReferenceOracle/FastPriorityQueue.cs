// ---------------------------------------------------------------------------
// REFERENCE ORACLE support — minimal reimplementation of the
// yohash.priorityqueue package (BlueRaja FastPriorityQueue lineage) that the
// reference EikonalSolver depends on. Behavioral fidelity notes:
//
// - 1-indexed backing array; a freshly constructed node has QueueIndex 0,
//   so Contains(node) on a NEW instance is ALWAYS false (reference-equality
//   check against _nodes[QueueIndex]). The reference EikonalSolver creates a
//   new FastLocation per neighbor visit, which means its
//   Contains → UpdatePriority path NEVER fires and duplicate entries are
//   enqueued instead — stale pops are effectively no-ops thanks to the
//   monotone φ guard. This quirk is deliberately preserved: the oracle must
//   behave exactly like the shipped reference implementation.
// - Higher priority = smaller float (min-queue); ties keep heap order.
// - One deviation: the array GROWS when full instead of throwing, so heavy
//   duplicate enqueueing on larger grids can't crash the oracle.
// ---------------------------------------------------------------------------

namespace Yohash.PriorityQueue
{
  public class FastPriorityQueueNode
  {
    public float Priority { get; protected internal set; }
    public int QueueIndex { get; internal set; }
  }

  public class FastPriorityQueue<T> where T : FastPriorityQueueNode
  {
    private int _numNodes;
    private T[] _nodes;

    public FastPriorityQueue(int maxNodes)
    {
      _numNodes = 0;
      _nodes = new T[maxNodes + 1];
    }

    public int Count => _numNodes;

    public bool Contains(T node)
    {
      int qi = node.QueueIndex;
      if (qi < 0 || qi >= _nodes.Length) {
        return false;
      }
      // reference equality — matches BlueRaja semantics (see header)
      return ReferenceEquals(_nodes[qi], node);
    }

    public void Enqueue(T node, float priority)
    {
      node.Priority = priority;
      _numNodes++;
      if (_numNodes >= _nodes.Length) {
        System.Array.Resize(ref _nodes, _nodes.Length * 2);
      }
      _nodes[_numNodes] = node;
      node.QueueIndex = _numNodes;
      CascadeUp(node);
    }

    public T Dequeue()
    {
      var result = _nodes[1];
      if (_numNodes == 1) {
        _nodes[1] = null;
        _numNodes = 0;
        return result;
      }
      var formerLast = _nodes[_numNodes];
      _nodes[1] = formerLast;
      formerLast.QueueIndex = 1;
      _nodes[_numNodes] = null;
      _numNodes--;
      CascadeDown(formerLast);
      return result;
    }

    public void UpdatePriority(T node, float priority)
    {
      node.Priority = priority;
      CascadeUp(node);
      CascadeDown(node);
    }

    private void CascadeUp(T node)
    {
      int index = node.QueueIndex;
      while (index > 1) {
        int parentIndex = index >> 1;
        var parent = _nodes[parentIndex];
        if (parent.Priority <= node.Priority) {
          break;
        }
        _nodes[index] = parent;
        parent.QueueIndex = index;
        _nodes[parentIndex] = node;
        node.QueueIndex = parentIndex;
        index = parentIndex;
      }
    }

    private void CascadeDown(T node)
    {
      int index = node.QueueIndex;
      while (true) {
        int childLeft = index * 2;
        if (childLeft > _numNodes) {
          break;
        }
        int childRight = childLeft + 1;
        var child = _nodes[childLeft];
        if (childRight <= _numNodes && _nodes[childRight].Priority < child.Priority) {
          childLeft = childRight;
          child = _nodes[childRight];
        }
        if (node.Priority <= child.Priority) {
          break;
        }
        _nodes[index] = child;
        child.QueueIndex = index;
        _nodes[childLeft] = node;
        node.QueueIndex = childLeft;
        index = childLeft;
      }
    }
  }
}
