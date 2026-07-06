using System.Runtime.CompilerServices;
using Unity.Collections;

namespace Yohash.ECSContinuumCrowds
{
  /// <summary>
  /// Indexed binary min-heap over caller-owned native arrays (spec §9.5
  /// option 1) — the Burst replacement for the repo's managed
  /// FastPriorityQueue, with exact FMM semantics including decrease-key
  /// (Contains → UpdatePriority, else Push).
  ///
  /// Storage (persistent per-group scratch, zero per-solve allocation):
  ///   Cells[slot] = cell id, Keys[slot] = priority (slot-aligned),
  ///   Pos[cellId] = slot or −1 when absent. Capacity = cell count: with
  ///   decrease-key a cell occupies at most one slot.
  /// </summary>
  public struct IndexedMinHeap
  {
    public NativeArray<int> Cells;
    public NativeArray<float> Keys;
    public NativeArray<int> Pos;
    public int Count;

    /// <summary>O(cellCount): empties the heap and clears all Pos entries.</summary>
    public void Reset()
    {
      Count = 0;
      for (int i = 0; i < Pos.Length; i++) {
        Pos[i] = -1;
      }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool Contains(int cell) => Pos[cell] >= 0;

    public void Push(int cell, float key)
    {
      int slot = Count++;
      Cells[slot] = cell;
      Keys[slot] = key;
      Pos[cell] = slot;
      SiftUp(slot);
    }

    /// <summary>Repo FastPriorityQueue behavior: decrease OR increase; re-heapify both ways.</summary>
    public void UpdatePriority(int cell, float key)
    {
      int slot = Pos[cell];
      Keys[slot] = key;
      slot = SiftUp(slot);
      SiftDown(slot);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void PushOrUpdate(int cell, float key)
    {
      if (Contains(cell)) {
        UpdatePriority(cell, key);
      } else {
        Push(cell, key);
      }
    }

    public int PopMin()
    {
      int cell = Cells[0];
      Pos[cell] = -1;
      Count--;
      if (Count > 0) {
        Cells[0] = Cells[Count];
        Keys[0] = Keys[Count];
        Pos[Cells[0]] = 0;
        SiftDown(0);
      }
      return cell;
    }

    private int SiftUp(int slot)
    {
      while (slot > 0) {
        int parent = (slot - 1) >> 1;
        if (Keys[slot] >= Keys[parent]) {
          break;
        }
        Swap(slot, parent);
        slot = parent;
      }
      return slot;
    }

    private void SiftDown(int slot)
    {
      while (true) {
        int left = slot * 2 + 1;
        if (left >= Count) {
          break;
        }
        int right = left + 1;
        int smallest = (right < Count && Keys[right] < Keys[left]) ? right : left;
        if (Keys[smallest] >= Keys[slot]) {
          break;
        }
        Swap(slot, smallest);
        slot = smallest;
      }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void Swap(int a, int b)
    {
      (Cells[a], Cells[b]) = (Cells[b], Cells[a]);
      (Keys[a], Keys[b]) = (Keys[b], Keys[a]);
      Pos[Cells[a]] = a;
      Pos[Cells[b]] = b;
    }
  }
}
