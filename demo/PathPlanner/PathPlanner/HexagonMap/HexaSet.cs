using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace PathPlanner.Hexagonal
{
    class HexaSet
    {
        List<HexaPos> _set;
        bool _allowDuplicate;

        public int Count
        {
            get
            {
                return _set.Count;
            }
        }

        public List<HexaPos> set
        {
            get
            {
                return _set;
            }
        }

        public HexaSet(bool allowDuplicate = false)
        {
            _set = new List<HexaPos>();
            _allowDuplicate = allowDuplicate;
        }

        public void AddPos(HexaPos pos)
        {
            if (false == HasPos(pos) && false == _allowDuplicate)
            {
                _set.Add(pos);
            }
        }

        public void AddPos(List<HexaPos> poses)
        {
            List<HexaPos>.Enumerator e = poses.GetEnumerator();
            while (e.MoveNext())
            {
                AddPos(e.Current);
            }
        }

        public void RemovePos(HexaPos pos)
        {
            List<HexaPos>.Enumerator e = _set.GetEnumerator();
            while (e.MoveNext())
            {
                if (e.Current.X == pos.X && e.Current.Y == pos.Y)
                {
                    _set.Remove(e.Current);
                    if(false == _allowDuplicate)
                        return;
                }
            }
        }

        public bool HasPos(HexaPos pos)
        {
            List<HexaPos>.Enumerator e = _set.GetEnumerator();
            while (e.MoveNext())
            {
                if (e.Current.X == pos.X && e.Current.Y == pos.Y)
                {
                    return true;
                }
            }
            return false;
        }

        public int GetPosCnt(HexaPos pos)
        {
            int cnt = 0;
            List<HexaPos>.Enumerator e = _set.GetEnumerator();
            while (e.MoveNext())
            {
                if (e.Current.X == pos.X && e.Current.Y == pos.Y)
                {
                    cnt++;
                }
            }
            return cnt;
        }

        public void Union(HexaSet otherSet)
        {
            List<HexaPos>.Enumerator e = otherSet.set.GetEnumerator();

            while (e.MoveNext())
            {
                AddPos(e.Current);
            }
        }

        public void Complement(HexaSet otherSet)
        {
            List<HexaPos>.Enumerator e = otherSet.set.GetEnumerator();

            while (e.MoveNext())
            {
                RemovePos(e.Current);
            }
        }

    }
}
