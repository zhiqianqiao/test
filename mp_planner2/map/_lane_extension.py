from tsmap import TSMap, Lane, Point3d, Bound
from Queue import Queue


class LaneSeg:
    def __init__(self, lane):
        self.lane = lane
        self.s = 0
        self.e = len(lane.ref_pts) - 1

    def __init__(self, lane, sref):
        self.lane = lane
        self.s = sref
        self.e = len(lane.ref_pts) - 1

    def __init__(self, lane, sref, eref):
        if sref < 0:
            sref = len(lane.ref_pts) + sref
        if eref < 0:
            eref = len(lane.ref_pts) + eref

        self.lane = lane
        self.s = sref
        self.e = eref

    def __str__(self):
        ret = "Seg = { id:"
        ret += str(self.lane.id)
        ret += " s:" + str(self.s)
        ret += " e:" + str(self.e)
        ret += " }"
        return ret

    def contains_loc(self, loc, tolerance=0.2):
        ref = self.lane.get_ref_pt(loc)
        sp = self.lane.ref_pts[self.s]
        ep = self.lane.ref_pts[self.e]
        # dist1=(sp.x-loc.x)**2+(sp.y-loc.y)**2
        # dist2=(ep.x-loc.x)**2+(ep.y-loc.y)**2
        ret = self.lane.compare(ref, sp)>0 and self.lane.compare(ep, ref)>0
        return ret


class LaneExtension:
    def __init__(self):
        self.segs = []
        self.lane2seg = {}

    def __str__(self):
        ret = "Ext=[[ \n"
        for seg in self.segs:
            ret += '  '
            ret += str(seg)
            ret += '\n'
        ret += "]]"
        return ret

    def add_seg(self, lane, sref, eref):
        # Already exist
        if lane in self.lane2seg:
            seg = self.lane2seg[lane]
            seg.s = min(sref, seg.s)
            seg.e = max(eref, seg.e)
            return

        # Add new seg
        seg = LaneSeg(lane, sref, eref)
        self.segs.append(seg)
        self.lane2seg[lane] = seg

    def contains_loc(self, lane, loc):
        if lane in self.lane2seg:
            seg = self.lane2seg[lane]
            return seg.contains_loc(loc)
        else:
            return False
