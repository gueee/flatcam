# ##########################################################
# FlatCAM: 2D Post-processing for Manufacturing            #
# File Author: Marius Adrian Stanciu (c)                   #
# Date: 3/10/2019                                          #
# MIT Licence                                              #
# ##########################################################

from appParsers.ParseDXF_Spline import spline2Polyline, normalize_2
from appParsers.ParseDXF_Spline import Vector as DxfVector

from shapely import LineString, Point, Polygon
from shapely.affinity import rotate, translate, scale
# from ezdxf.math import Vector as ezdxf_vector
from ezdxf.math import Vec3 as ezdxf_vector

import math

import logging

log = logging.getLogger('base2')

# Explicit import for version checking
import ezdxf

# Try to import text2path addon (available in ezdxf >= 0.16.0)
# Graceful degradation handles edge case where ezdxf is installed but addon is missing/corrupted
try:
    from ezdxf.addons import text2path
    HAS_TEXT2PATH = True
except ImportError:
    HAS_TEXT2PATH = False
    text2path = None  # Prevent NameError in downstream code
    log.warning("ezdxf text2path addon not available. "
                "DXF TEXT/MTEXT/ATTRIB entities will be skipped. "
                "Upgrade ezdxf to >= 0.16.0 for text support.")

# Separate version check with its own error handling
# This prevents version parsing errors from affecting the import flag
HAS_TEXT2PATH_VERSION_OK = True
if HAS_TEXT2PATH:
    try:
        EZDXF_VERSION = tuple(map(int, ezdxf.__version__.split('.')[:3]))
        if EZDXF_VERSION < (0, 16, 0):
            HAS_TEXT2PATH_VERSION_OK = False
            log.warning(f"ezdxf version {ezdxf.__version__} detected. "
                        f"text2path requires >= 0.16.0. Text import may not work correctly.")
    except (ValueError, AttributeError) as e:
        # Version string format unexpected - log warning but continue
        HAS_TEXT2PATH_VERSION_OK = False
        log.warning(f"Could not parse ezdxf version string: {e}. "
                    f"Text import may not work correctly.")


def distance(pt1, pt2):
    return math.sqrt((pt1[0] - pt2[0]) ** 2 + (pt1[1] - pt2[1]) ** 2)


def dxfpoint2shapely(point):

    geo = Point(point.dxf.location).buffer(0.01)
    return geo


def dxfline2shapely(line):

    try:
        start = (line.dxf.start[0], line.dxf.start[1])
        stop = (line.dxf.end[0], line.dxf.end[1])

    except Exception as e:
        log.error(str(e))
        return None

    geo = LineString([start, stop])

    return geo


def dxfcircle2shapely(circle, n_points=100):

    ocs = circle.ocs()
    # if the extrusion attribute is not (0, 0, 1) then we have to change the coordinate system from OCS to WCS
    # Use tolerance-based comparison to handle floating-point precision issues in DXF files
    ext = circle.dxf.extrusion
    if not (math.isclose(ext[0], 0.0, abs_tol=1e-9) and
            math.isclose(ext[1], 0.0, abs_tol=1e-9) and
            math.isclose(ext[2], 1.0, abs_tol=1e-9)):
        center_pt = ocs.to_wcs(circle.dxf.center)
    else:
        center_pt = circle.dxf.center

    radius = circle.dxf.radius
    geo = Point(center_pt).buffer(radius, int(n_points / 4))

    return geo


def dxfarc2shapely(arc, n_points=100):
    # ocs = arc.ocs()
    # # if the extrusion attribute is not (0, 0, 1) then we have to change the coordinate system from OCS to WCS
    # if arc.dxf.extrusion != (0, 0, 1):
    #     arc_center = ocs.to_wcs(arc.dxf.center)
    #     start_angle = math.radians(arc.dxf.start_angle) + math.pi
    #     end_angle = math.radians(arc.dxf.end_angle) + math.pi
    #     dir = 'CW'
    # else:
    #     arc_center = arc.dxf.center
    #     start_angle = math.radians(arc.dxf.start_angle)
    #     end_angle = math.radians(arc.dxf.end_angle)
    #     dir = 'CCW'
    #
    # center_x = arc_center[0]
    # center_y = arc_center[1]
    # radius = arc.dxf.radius
    #
    # point_list = []
    #
    # if start_angle > end_angle:
    #     start_angle +=  2 * math.pi
    #
    # line_seg = int((n_points * (end_angle - start_angle)) / math.pi)
    # step_angle = (end_angle - start_angle) / float(line_seg)
    #
    # angle = start_angle
    # for step in range(line_seg + 1):
    #     if dir == 'CCW':
    #         x = center_x + radius * math.cos(angle)
    #         y = center_y + radius * math.sin(angle)
    #     else:
    #         x = center_x + radius * math.cos(-angle)
    #         y = center_y + radius * math.sin(-angle)
    #     point_list.append((x, y))
    #     angle += step_angle
    #
    #
    # log.debug("X = %.4f, Y = %.4f, Radius = %.4f, start_angle = %.1f, stop_angle = %.1f, step_angle = %.4f, dir=%s" %
    #           (center_x, center_y, radius, start_angle, end_angle, step_angle, dir))
    #
    # geo = LineString(point_list)
    # return geo

    ocs = arc.ocs()
    # if the extrusion attribute is not (0, 0, 1) then we have to change the coordinate system from OCS to WCS
    # Use tolerance-based comparison to handle floating-point precision issues in DXF files
    ext = arc.dxf.extrusion
    if not (math.isclose(ext[0], 0.0, abs_tol=1e-9) and
            math.isclose(ext[1], 0.0, abs_tol=1e-9) and
            math.isclose(ext[2], 1.0, abs_tol=1e-9)):
        arc_center = ocs.to_wcs(arc.dxf.center)
        start_angle = arc.dxf.start_angle + 180
        end_angle = arc.dxf.end_angle + 180
        direction = 'CW'
    else:
        arc_center = arc.dxf.center
        start_angle = arc.dxf.start_angle
        end_angle = arc.dxf.end_angle
        direction = 'CCW'

    center_x = arc_center[0]
    center_y = arc_center[1]
    radius = arc.dxf.radius

    point_list = []

    if start_angle > end_angle:
        start_angle = start_angle - 360
    angle = start_angle

    step_angle = float(abs(end_angle - start_angle) / n_points)

    while angle <= end_angle:
        if direction == 'CCW':
            x = center_x + radius * math.cos(math.radians(angle))
            y = center_y + radius * math.sin(math.radians(angle))
        else:
            x = center_x + radius * math.cos(math.radians(-angle))
            y = center_y + radius * math.sin(math.radians(-angle))
        point_list.append((x, y))
        angle += abs(step_angle)

    # in case the number of segments do not cover everything until the end of the arc
    if angle != end_angle:
        if direction == 'CCW':
            x = center_x + radius * math.cos(math.radians(end_angle))
            y = center_y + radius * math.sin(math.radians(end_angle))
        else:
            x = center_x + radius * math.cos(math.radians(- end_angle))
            y = center_y + radius * math.sin(math.radians(- end_angle))
        point_list.append((x, y))

    # log.debug("X = %.4f, Y = %.4f, Radius = %.4f, start_angle = %.1f, stop_angle = %.1f, step_angle = %.4f" %
    #           (center_x, center_y, radius, start_angle, end_angle, step_angle))

    geo = LineString(point_list)
    return geo


def dxfellipse2shapely(ellipse, ellipse_segments=100):
    # center = ellipse.dxf.center
    # start_angle = ellipse.dxf.start_param
    # end_angle = ellipse.dxf.end_param

    ocs = ellipse.ocs()
    # if the extrusion attribute is not (0, 0, 1) then we have to change the coordinate system from OCS to WCS
    # Use tolerance-based comparison to handle floating-point precision issues in DXF files
    ext = ellipse.dxf.extrusion
    if not (math.isclose(ext[0], 0.0, abs_tol=1e-9) and
            math.isclose(ext[1], 0.0, abs_tol=1e-9) and
            math.isclose(ext[2], 1.0, abs_tol=1e-9)):
        center = ocs.to_wcs(ellipse.dxf.center)
        # start_param and end_param are scalar angle values (radians), not coordinates - do not convert them
        start_angle = ellipse.dxf.start_param
        end_angle = ellipse.dxf.end_param
        direction = 'CW'
    else:
        center = ellipse.dxf.center
        start_angle = ellipse.dxf.start_param
        end_angle = ellipse.dxf.end_param
        direction = 'CCW'

    # print("Dir = %s" % dir)
    major_axis = ellipse.dxf.major_axis
    ratio = ellipse.dxf.ratio

    points_list = []
    major_axis = DxfVector(list(major_axis))

    major_x = major_axis[0]
    major_y = major_axis[1]

    if start_angle >= end_angle:
        end_angle += 2.0 * math.pi

    line_seg = int((ellipse_segments * (end_angle - start_angle)) / math.pi)
    step_angle = abs(end_angle - start_angle) / float(line_seg)

    angle = start_angle
    for step in range(line_seg + 1):
        if direction == 'CW':
            major_dim = normalize_2(major_axis)
            minor_dim = normalize_2(DxfVector([ratio * k for k in major_axis]))
            vx = (major_dim[0] + major_dim[1]) * math.cos(angle)
            vy = (minor_dim[0] - minor_dim[1]) * math.sin(angle)
            x = center[0] + major_x * vx - major_y * vy
            y = center[1] + major_y * vx + major_x * vy
            angle += step_angle
        else:
            major_dim = normalize_2(major_axis)
            minor_dim = (DxfVector([ratio * k for k in major_dim]))
            vx = (major_dim[0] + major_dim[1]) * math.cos(angle)
            vy = (minor_dim[0] + minor_dim[1]) * math.sin(angle)
            x = center[0] + major_x * vx + major_y * vy
            y = center[1] + major_y * vx + major_x * vy
            angle += step_angle

        points_list.append((x, y))

    geo = LineString(points_list)
    return geo


def dxfpolyline2shapely(polyline):
    final_pts = []
    pts = polyline.points()
    for i in pts:
        final_pts.append((i[0], i[1]))
    if polyline.is_closed:
        final_pts.append(final_pts[0])

    geo = LineString(final_pts)
    return geo


def dxflwpolyline2shapely(lwpolyline):
    final_pts = []

    for point in lwpolyline:
        x, y, _, _, _ = point
        final_pts.append((x, y))
    if lwpolyline.closed:
        final_pts.append(final_pts[0])

    geo = LineString(final_pts)
    return geo


def dxfsolid2shapely(solid):
    iterator = 0
    corner_list = []
    try:
        corner_list.append(solid[iterator])
        iterator += 1
    except Exception:
        return Polygon(corner_list)


def dxfspline2shapely(spline):
    # for old version of ezdxf
    # with spline.edit_data() as spline_data:
    #     ctrl_points = spline_data.control_points
    #     try:
    #         # required if using old version of ezdxf
    #         knot_values = spline_data.knot_values
    #     except AttributeError:
    #         knot_values = spline_data.knots

    ctrl_points = spline.control_points
    knot_values = spline.knots
    is_closed = spline.closed
    degree = spline.dxf.degree

    x_list, y_list, _ = spline2Polyline(ctrl_points, degree=degree, closed=is_closed, segments=20, knots=knot_values)
    points_list = zip(x_list, y_list)

    geo = LineString(points_list)
    return geo


def dxftrace2shapely(trace):
    """
    Convert DXF TRACE entity to Shapely Polygon.
    
    :param trace: ezdxf TRACE entity
    :return: Shapely Polygon
    """
    corner_list = []
    try:
        for point in trace:
            corner_list.append(point)
    except Exception:
        pass
    
    if len(corner_list) < 3:
        # Return empty polygon if not enough points
        return Polygon()
    
    return Polygon(corner_list)


def dxftext2shapely(text_entity, text_mode='stroke'):
    """
    Convert DXF TEXT, MTEXT, ATTRIB, or ATTDEF entity to Shapely geometry.
    
    :param text_entity: ezdxf TEXT, MTEXT, ATTRIB, or ATTDEF entity
    :param text_mode: 'stroke' for single-line paths (CNC engraving),
                      'outline' for filled polygon outlines,
                      'none' to skip conversion (returns empty list)
    :return: List of Shapely geometry objects (LineString or Polygon).
             Returns empty list on error, when text_mode='none', or when conversion fails.
    """
    # Use module-level imports for LineString and Polygon
    
    geometries = []
    
    # Check if text2path is available and version is compatible
    if not HAS_TEXT2PATH:
        log.warning("text2path addon not available, skipping text conversion")
        return []
    
    # Check if ezdxf version is compatible
    if not HAS_TEXT2PATH_VERSION_OK:
        log.warning("ezdxf version may be incompatible, skipping text conversion")
        return []
    
    # Validate text_mode parameter
    if text_mode not in ('stroke', 'outline', 'none'):
        log.warning(f"Invalid text_mode '{text_mode}', using 'stroke'")
        text_mode = 'stroke'
    
    # Skip conversion if mode is 'none'
    if text_mode == 'none':
        return []
    
    # Initialize layer for error handling before try block
    layer = 'unknown'
    
    try:
        # Get text content - TEXT, MTEXT, and ATTRIB entities all use 'text' attribute
        # REVIEW FIX v1.7: ATTRIB does NOT use 'value' - it uses 'text' like other entities
        text_content = getattr(text_entity.dxf, 'text', '')
        
        # Get insert point for logging (may be in OCS, we use as-is - see Limitations)
        insert_point = getattr(text_entity.dxf, 'insert', None)
        
        # Get layer for better error messages
        layer = getattr(text_entity.dxf, 'layer', 'unknown')
        
        # Convert text entity to paths using ezdxf text2path addon
        # REVIEW FIX v1.7: Correct API is make_paths_from_entity(), NOT text2paths()
        # text2paths() does NOT exist - this was a bug in plan v1.6
        paths = list(text2path.make_paths_from_entity(text_entity))
        
        for path in paths:
            # Get vertices from the path using flattening()
            # REVIEW FIX v1.7: flattening() REQUIRES distance argument
            # Signature: Path.flattening(distance: float, segments: int = 4) -> Iterator[Vec3]
            # Use distance=0.01 for reasonable CNC precision
            vertices_3d = list(path.flattening(distance=0.01))
            
            # Use .x, .y attributes instead of indexing for safety
            vertices = [(v.x, v.y) for v in vertices_3d]
            
            if len(vertices) < 2:
                continue
            
            # Create appropriate Shapely geometry based on mode and path type
            if text_mode == 'stroke' or not path.is_closed:
                # Single-line path for CNC engraving
                geometries.append(LineString(vertices))
            elif path.is_closed and len(vertices) >= 3:
                # Closed outline for filled text
                geometries.append(Polygon(vertices))
            else:
                # Fallback to LineString
                geometries.append(LineString(vertices))
        
        if geometries:
            # Truncate text for logging to avoid excessive length
            text_preview = str(text_content)[:50] if len(str(text_content)) > 50 else str(text_content)
            log.debug(f"Converted TEXT '{text_preview}' at {insert_point} "
                      f"to {len(geometries)} geometry objects")
        else:
            text_preview = str(text_content)[:50] if len(str(text_content)) > 50 else str(text_content)
            log.debug(f"TEXT '{text_preview}' produced no geometry")
        
    except AttributeError as e:
        log.error(f"Failed to convert {text_entity.dxftype()} on layer "
                  f"'{layer}' - missing attribute: {e}")
        return []
    except Exception as e:
        log.error(f"Failed to convert {text_entity.dxftype()} on layer "
                  f"'{layer}' to geometry: {type(e).__name__}: {e}")
        return []
    
    return geometries


def getdxfgeo(dxf_object, text_mode='stroke', units=None):
    """
    Extract all geometry and text from DXF modelspace.
    
    :param dxf_object: ezdxf Document object
    :param text_mode: 'stroke' or 'outline' for text conversion, 'none' to skip
    :param units: Document units (optional, currently unused)
    :return: Combined list of Shapely geometry objects
    """
    msp = dxf_object.modelspace()
    geos = get_geo(dxf_object, msp, text_mode=text_mode)
    
    return geos


def get_geo_from_insert(dxf_object, insert, text_mode='stroke'):
    """
    Extract geometry from INSERT (block reference) entity.
    
    :param dxf_object: ezdxf Document object
    :param insert: INSERT entity
    :param text_mode: 'stroke', 'outline', or 'none' for text conversion
    :return: List of transformed Shapely geometry objects
    """
    # NO NEW IMPORTS NEEDED - translate, scale, rotate already imported from shapely.affinity
    
    geo_block_transformed = []
    
    try:
        phi = insert.dxf.rotation
        tr = insert.dxf.insert
        sx = insert.dxf.xscale
        sy = insert.dxf.yscale
        r_count = insert.dxf.row_count
        r_spacing = insert.dxf.row_spacing
        c_count = insert.dxf.column_count
        c_spacing = insert.dxf.column_spacing
        
        # identify the block given the 'INSERT' type entity name
        # NOTE: Use direct indexing (not .get()) to match existing code pattern
        block = dxf_object.blocks[insert.dxf.name]
        block_coords = (block.block.dxf.base_point[0], block.block.dxf.base_point[1])
        
        # ====================================================================
        # CRITICAL FIX: FORWARD text_mode parameter for text-in-block support
        # This is the ONLY logic change - all transformation code preserved
        # ====================================================================
        geo_block = get_geo(dxf_object, block, text_mode=text_mode)
        
        if not geo_block:
            return []
        
        # iterate over the geometries found and apply any transformation 
        # found in the 'INSERT' entity attributes
        for geo in geo_block:
            # get the bounds of the geometry
            # minx, miny, maxx, maxy = geo.bounds
            
            if tr[0] != 0 or tr[1] != 0:
                geo = translate(geo, (tr[0] - block_coords[0]), (tr[1] - block_coords[1]))
            
            if sx != 1 or sy != 1:
                geo = scale(geo, sx, sy)
            if phi != 0:
                origin = tr
                if isinstance(origin, str) and origin.lower() == 'c':
                    origin = 'center'
                elif isinstance(origin, ezdxf_vector):
                    origin = list(origin)
                geo = rotate(geo, phi, origin=origin)
            
            # support for array block insertions; the rows are spaced along Y and the columns along X.
            # An insert that is not an array has a single row and a single column.
            for row in range(max(r_count, 1)):
                for col in range(max(c_count, 1)):
                    geo_block_transformed.append(translate(geo, col * c_spacing, row * r_spacing))
        
        log.debug(f"INSERT block '{insert.dxf.name}' converted {len(geo_block)} entities")
        
    except KeyError as e:
        log.error(f"INSERT block not found: {e}")
        return []
    except Exception as e:
        log.error(f"Failed to process INSERT entity: {type(e).__name__}: {e}")
        return []
    
    return geo_block_transformed


def get_geo(dxf_object, container, text_mode='stroke'):
    """
    Extract geometry from DXF container.
    
    :param dxf_object: ezdxf Document object
    :param container: DXF container (e.g., modelspace, block)
    :param text_mode: 'stroke', 'outline', or 'none' for text conversion
    :return: List of Shapely geometry objects
    """
    # store shapely geometry here
    geo = []

    for dxf_entity in container:
        g = []
        # print("Entity", dxf_entity.dxftype())
        if dxf_entity.dxftype() == 'POINT':
            g = dxfpoint2shapely(dxf_entity,)
        elif dxf_entity.dxftype() == 'LINE':
            g = dxfline2shapely(dxf_entity,)
        elif dxf_entity.dxftype() == 'CIRCLE':
            g = dxfcircle2shapely(dxf_entity)
        elif dxf_entity.dxftype() == 'ARC':
            g = dxfarc2shapely(dxf_entity)
        elif dxf_entity.dxftype() == 'ELLIPSE':
            g = dxfellipse2shapely(dxf_entity)
        elif dxf_entity.dxftype() == 'LWPOLYLINE':
            g = dxflwpolyline2shapely(dxf_entity)
        elif dxf_entity.dxftype() == 'POLYLINE':
            g = dxfpolyline2shapely(dxf_entity)
        elif dxf_entity.dxftype() == 'SOLID':
            g = dxfsolid2shapely(dxf_entity)
        elif dxf_entity.dxftype() == 'TRACE':
            g = dxftrace2shapely(dxf_entity)
        elif dxf_entity.dxftype() == 'SPLINE':
            g = dxfspline2shapely(dxf_entity)
        # ============================================================
        # NEW: TEXT, MTEXT, ATTRIB and ATTDEF handling
        # ============================================================
        elif dxf_entity.dxftype() in ('TEXT', 'MTEXT', 'ATTRIB', 'ATTDEF'):
            # Check if text2path is available before attempting conversion
            if HAS_TEXT2PATH and text_mode != 'none':
                text_geos = dxftext2shapely(dxf_entity, text_mode=text_mode)
                if text_geos:
                    geo.extend(text_geos)
            elif not HAS_TEXT2PATH:
                log.debug("TEXT/MTEXT/ATTRIB/ATTDEF entity skipped - text2path addon not available")
            # Skip the common g append logic below - text handled inline
            # CRITICAL: continue prevents double-append of empty g list
            continue
        # ============================================================
        # UPDATED: INSERT handling - forward text_mode parameter
        # ============================================================
        elif dxf_entity.dxftype() == 'INSERT':
            g = get_geo_from_insert(dxf_object, dxf_entity, text_mode=text_mode)
        else:
            log.debug(" %s is not supported yet." % dxf_entity.dxftype())

        if g is not None:
            if type(g) == list:
                for subg in g:
                    geo.append(subg)
            else:
                geo.append(g)

    return geo


def getdxftext(exf_object, object_type, units=None):
    pass

# def get_geo_from_block(dxf_object):
#     geo_block_transformed = []
#
#     msp = dxf_object.modelspace()
#     # iterate through all 'INSERT' entities found in modelspace msp
#     for insert in msp.query('INSERT'):
#         phi = insert.dxf.rotation
#         tr = insert.dxf.insert
#         sx = insert.dxf.xscale
#         sy = insert.dxf.yscale
#         r_count = insert.dxf.row_count
#         r_spacing = insert.dxf.row_spacing
#         c_count = insert.dxf.column_count
#         c_spacing = insert.dxf.column_spacing
#
#         # print(phi, tr)
#
#         # identify the block given the 'INSERT' type entity name
#         print(insert.dxf.name)
#         block = dxf_object.blocks[insert.dxf.name]
#         block_coords = (block.block.dxf.base_point[0], block.block.dxf.base_point[1])
#
#         # get a list of geometries found in the block
#         # store shapely geometry here
#         geo_block = []
#
#         for dxf_entity in block:
#             g = []
#             # print("Entity", dxf_entity.dxftype())
#             if dxf_entity.dxftype() == 'POINT':
#                 g = dxfpoint2shapely(dxf_entity, )
#             elif dxf_entity.dxftype() == 'LINE':
#                 g = dxfline2shapely(dxf_entity, )
#             elif dxf_entity.dxftype() == 'CIRCLE':
#                 g = dxfcircle2shapely(dxf_entity)
#             elif dxf_entity.dxftype() == 'ARC':
#                 g = dxfarc2shapely(dxf_entity)
#             elif dxf_entity.dxftype() == 'ELLIPSE':
#                 g = dxfellipse2shapely(dxf_entity)
#             elif dxf_entity.dxftype() == 'LWPOLYLINE':
#                 g = dxflwpolyline2shapely(dxf_entity)
#             elif dxf_entity.dxftype() == 'POLYLINE':
#                 g = dxfpolyline2shapely(dxf_entity)
#             elif dxf_entity.dxftype() == 'SOLID':
#                 g = dxfsolid2shapely(dxf_entity)
#             elif dxf_entity.dxftype() == 'TRACE':
#                 g = dxftrace2shapely(dxf_entity)
#             elif dxf_entity.dxftype() == 'SPLINE':
#                 g = dxfspline2shapely(dxf_entity)
#             elif dxf_entity.dxftype() == 'INSERT':
#                 log.debug("Not supported yet.")
#             else:
#                 log.debug("Not supported yet.")
#
#             if g is not None:
#                 if type(g) == list:
#                     for subg in g:
#                         geo_block.append(subg)
#                 else:
#                     geo_block.append(g)
#
#         # iterate over the geometries found and apply any transformation found in the 'INSERT' entity attributes
#         for geo in geo_block:
#             if tr[0] != 0 or tr[1] != 0:
#                 geo = translate(geo, (tr[0] - block_coords[0]), (tr[1] - block_coords[1]))
#
#             # support for array block insertions
#             if r_count > 1:
#                 for r in range(r_count):
#                     geo_block_transformed.append(translate(geo, (tr[0] + (r * r_spacing) - block_coords[0]), 0))
#
#             if c_count > 1:
#                 for c in range(c_count):
#                     geo_block_transformed.append(translate(geo, 0, (tr[1] + (c * c_spacing) - block_coords[1])))
#
#             if sx != 1 or sy != 1:
#                 geo = scale(geo, sx, sy)
#             if phi != 0:
#                 geo = rotate(geo, phi, origin=tr)
#
#             geo_block_transformed.append(geo)
#     return geo_block_transformed
