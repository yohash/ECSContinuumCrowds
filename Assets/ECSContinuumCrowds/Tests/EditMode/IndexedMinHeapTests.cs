using System.Collections.Generic;
using NUnit.Framework;
using Unity.Collections;

namespace Yohash.ECSContinuumCrowds.Tests
{
  public class IndexedMinHeapTests
  {
    private static IndexedMinHeap MakeHeap(int capacity, out NativeArray<int> cells,
      out NativeArray<float> keys, out NativeArray<int> pos)
    {
      cells = new NativeArray<int>(capacity, Allocator.Temp);
      keys = new NativeArray<float>(capacity, Allocator.Temp);
      pos = new NativeArray<int>(capacity, Allocator.Temp);
      var heap = new IndexedMinHeap { Cells = cells, Keys = keys, Pos = pos };
      heap.Reset();
      return heap;
    }

    [Test]
    public void PopsInAscendingKeyOrder()
    {
      var heap = MakeHeap(16, out var cells, out var keys, out var pos);
      using (cells)
      using (keys)
      using (pos) {
        float[] priorities = { 5f, 1f, 9f, 3f, 7f, 2f, 8f, 0.5f };
        for (int i = 0; i < priorities.Length; i++) {
          heap.Push(i, priorities[i]);
        }
        float last = float.NegativeInfinity;
        while (heap.Count > 0) {
          int slotKeyOwner = heap.Cells[0];
          float key = heap.Keys[0];
          int popped = heap.PopMin();
          Assert.AreEqual(slotKeyOwner, popped);
          Assert.GreaterOrEqual(key, last);
          Assert.IsFalse(heap.Contains(popped));
          last = key;
        }
      }
    }

    [Test]
    public void DecreaseKeyReordersAndContainsTracks()
    {
      var heap = MakeHeap(8, out var cells, out var keys, out var pos);
      using (cells)
      using (keys)
      using (pos) {
        heap.Push(0, 10f);
        heap.Push(1, 20f);
        heap.Push(2, 30f);
        Assert.IsTrue(heap.Contains(2));
        Assert.IsFalse(heap.Contains(3));

        heap.UpdatePriority(2, 5f); // decrease-key to the front
        Assert.AreEqual(2, heap.PopMin());
        Assert.AreEqual(0, heap.PopMin());
        Assert.AreEqual(1, heap.PopMin());
      }
    }

    [Test]
    public void PushOrUpdateMirrorsRepoQueueBehavior()
    {
      var heap = MakeHeap(8, out var cells, out var keys, out var pos);
      using (cells)
      using (keys)
      using (pos) {
        heap.PushOrUpdate(3, 7f);   // absent → push
        heap.PushOrUpdate(3, 2f);   // present → update, no duplicate
        Assert.AreEqual(1, heap.Count);
        Assert.AreEqual(3, heap.PopMin());
        Assert.AreEqual(0, heap.Count);
      }
    }

    [Test]
    public void RandomStressAgainstManagedReference()
    {
      const int capacity = 512;
      var heap = MakeHeap(capacity, out var cells, out var keys, out var pos);
      using (cells)
      using (keys)
      using (pos) {
        var rng = new Unity.Mathematics.Random(424242);
        var reference = new Dictionary<int, float>();

        for (int round = 0; round < 5000; round++) {
          int op = rng.NextInt(0, 3);
          if (op == 0 && reference.Count < capacity) {
            int cell = rng.NextInt(0, capacity);
            float key = rng.NextFloat(0f, 100f);
            if (reference.ContainsKey(cell)) {
              heap.UpdatePriority(cell, key);
            } else {
              heap.Push(cell, key);
            }
            reference[cell] = key;
          } else if (op == 1 && reference.Count > 0) {
            float min = float.PositiveInfinity;
            foreach (var kv in reference) {
              if (kv.Value < min) min = kv.Value;
            }
            float popKey = heap.Keys[0];
            int popped = heap.PopMin();
            Assert.AreEqual(min, popKey, 0f, "heap min key diverged from reference");
            Assert.AreEqual(min, reference[popped], 0f, "popped cell doesn't hold the min key");
            reference.Remove(popped);
          } else if (reference.Count > 0) {
            // random re-prioritization of an existing cell
            foreach (var kv in reference) {
              float key = rng.NextFloat(0f, 100f);
              heap.UpdatePriority(kv.Key, key);
              reference[kv.Key] = key;
              break;
            }
          }
          Assert.AreEqual(reference.Count, heap.Count);
        }
      }
    }
  }
}
