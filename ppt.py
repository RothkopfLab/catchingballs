import viz

# import add-on
vrpn = viz.add('vrpn7.dle')


def add_tracker(marker=0):
    """
    Add a tracker to the vrpn object
    :param marker: marker number
    :return: the new tracker
    """
    tracker = vrpn.addTracker('PPT0@WV', marker)
    return tracker


def link(tracker, target, ori=None, offset=[0, 0, 0], pre_trans=[0, 0, 0]):
    """
    Link the tracker to a target (with an optional rotation sensor ori)
    :param tracker: tracker to be linked
    :param target: target to link to
    :param ori: orientation tracker
    :param offset: offset from tracker to target
    :param pre_trans: pre multiplied with input matrix
    :return: link object
    """
    if ori:
        # merge orientation and position trackers
        merged_trackers = viz.mergeLinkable(pos=tracker, ori=ori)
    else:
        merged_trackers = tracker

    # link to target (usually viz.MainView or an object like a glove)
    complete_link = viz.link(merged_trackers, target)
    complete_link.setOffset(offset)

    # distance between tracker and actual position of target
    complete_link.preTrans(pre_trans)
    # TODO: maybe different solution (offsets and stuff)
    return complete_link
