"""Shared physical geometry and inventory for the warehouse world and runtime."""
import math


def inventory(config):
    z = config['support_height'] + config['box_size'][2] / 2
    return [dict(id=f"cargo_{item['rack']}_{item['color']}",rack=item['rack'],color=item['color'],
                 position=[config['racks'][item['rack']][0],config['racks'][item['rack']][1],z])
            for item in config['inventory']]


def dock(config,position,center=None):
    x=position[0]-config['dock_offset']
    if center is not None:x=min(x,center[0]-config['dock_offset'])
    return [x,position[1]]


def fixture_parts(config):
    """Name, absolute center, dimensions; every part has physical collision."""
    parts = []
    centers = {**config['racks'], **{k:v['center'] for k,v in config['outputs'].items()}}
    for name, (x,y) in centers.items():
        ix,iy,ih=config['container_inner'];ox,oy,oh=config['container_outer']
        floor=config['support_height'];bottom=oh-ih;wx=(ox-ix)/2;wy=(oy-iy)/2
        # Container bottom rests directly on the ground; internal floor is 3.8 mm high.
        parts.append((f'{name}_support',[x,y,floor-bottom/2],[ox,oy,bottom]))
        for side in [-1,1]:
            parts.append((f'{name}_end_{side}',[x+side*(ix/2+wx/2),y,floor+ih/2],[wx,oy,ih]))
            parts.append((f'{name}_side_{side}',[x,y+side*(iy/2+wy/2),floor+ih/2],[ix,wy,ih]))
    xmin,xmax,ymin,ymax=config['room']
    height=config.get('wall_height',.12)
    for name, center, size in [
        ('wall_west', [xmin,(ymin+ymax)/2,height/2], [.08,ymax-ymin,height]),
        ('wall_east', [xmax,(ymin+ymax)/2,height/2], [.08,ymax-ymin,height]),
        ('wall_south', [(xmin+xmax)/2,ymin,height/2], [xmax-xmin,.08,height]),
        ('wall_north', [(xmin+xmax)/2,ymax,height/2], [xmax-xmin,.08,height])]:
        parts.append((name,center,size))
    return parts


def planning_parts(config):
    # Gazebo's ground plane already supplies physical contacts. Include its
    # surface in arm planning, with 2 mm tolerance for wheel contact jitter.
    xmin,xmax,ymin,ymax=config['room']
    return [*fixture_parts(config),('_warehouse_ground',
            [(xmin+xmax)/2,(ymin+ymax)/2,-.012],[xmax-xmin,ymax-ymin,.02])]


def slot(config, output, index):
    if output not in config['outputs'] or not 0 <= index < len(config['output_offsets']):
        raise ValueError('unknown output or output capacity exhausted')
    x,y=config['outputs'][output]['center']
    return [x+config['output_offsets'][index], y,
            config['support_height']+config['box_size'][2]/2]


def inside(config, output, position):
    x,y=config['outputs'][output]['center']
    # Require the full box footprint inside the tray and resting on its base.
    bx,by,bz=config['box_size']
    return (abs(position[0]-x) <= config['container_inner'][0]/2-bx/2+.001 and
            abs(position[1]-y) <= config['container_inner'][1]/2-by/2+.001 and
            abs(position[2]-(config['support_height']+bz/2)) < .010)


def validate(config):
    goods=inventory(config)
    bottom=config['container_outer'][2]-config['container_inner'][2]
    if not math.isclose(config['support_height'],bottom,abs_tol=1e-9):
        raise ValueError('container outer bottom must contact the ground')
    if len(config['racks'])!=6 or len(config['outputs'])!=3:
        raise ValueError('warehouse requires six racks and three outputs')
    if len(goods)!=5 or len({g['rack'] for g in goods})!=5:
        raise ValueError('five cargo must occupy five distinct source boxes')
    if any(o<=i for o,i in zip(config['container_outer'],config['container_inner'])):
        raise ValueError('outer dimensions must exceed inner dimensions')
    if any(b>=i for b,i in zip(config['box_size'],config['container_inner'])):
        raise ValueError('cargo must fit inside container with clearance')
    colors=[v['color'] for v in config['outputs'].values()]
    if len(set(colors))!=len(colors) or set(colors)!=set(config['colors']):
        raise ValueError('each color must have one unique destination')
    for color in config['colors']:
        count=sum(g['color']==color for g in goods)
        if count>len(config['output_offsets']):raise ValueError('output capacity insufficient')
    points=[g['position'] for g in goods]
    for i,p in enumerate(points):
        for q in points[i+1:]:
            if math.dist(p[:2],q[:2]) < .06:raise ValueError('initial cargo overlap or insufficient spacing')
    for output in config['outputs']:
        for i in range(len(config['output_offsets'])):
            if not inside(config,output,slot(config,output,i)):raise ValueError('slot outside physical tray')
    return goods
