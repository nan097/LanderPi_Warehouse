#!/usr/bin/env python3
"""Generate warehouse SDF, conservative occupancy map and Nav2 parameters."""
from pathlib import Path
import sys
import math
import xml.etree.ElementTree as ET
import yaml
from PIL import Image, ImageDraw, ImageFont

ROOT=Path(__file__).resolve().parents[1]
SHARE=ROOT/'src/landerpi_simulation'
sys.path.insert(0,str(SHARE/'scripts'))
from warehouse_layout import fixture_parts, validate


def child(parent,tag,text=None,**attrs):
    e=ET.SubElement(parent,tag,attrs)
    if text is not None:e.text=str(text)
    return e


def geometry(parent,size):
    child(child(child(parent,'geometry'),'box'),'size',' '.join(map(str,size)))


def material(visual,rgb):
    m=child(visual,'material')
    child(m,'ambient',' '.join(str(.5*c) for c in rgb)+' 1')
    child(m,'diffuse',' '.join(map(str,rgb))+' 1')
    child(m,'specular','0 0 0 1')


def layout_svg(config):
    xmin,xmax,ymin,ymax=config['room'];scale=250;pad=90
    width=int((xmax-xmin)*scale+pad*2);height=int((ymax-ymin)*scale+pad*2)
    def point(p):return pad+(p[0]-xmin)*scale,pad+(ymax-p[1])*scale
    svg=[f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
         '<rect width="100%" height="100%" fill="#f4f6f8"/>',
         f'<text x="{width/2}" y="32" text-anchor="middle" font-family="sans-serif" font-size="20">Ground boxes — 2.4 × 2.0 m — 5 cargo</text>',
         f'<rect x="{pad}" y="{pad}" width="{(xmax-xmin)*scale}" height="{(ymax-ymin)*scale}" fill="white" stroke="#596779" stroke-width="{.08*scale}"/>']
    goods=validate(config);centers={**config['racks'],**{k:v['center'] for k,v in config['outputs'].items()}}
    ox,oy,_=config['container_outer']
    for name,p in centers.items():
        x,y=point(p);color=config['outputs'].get(name,{}).get('color');fill=color if color else '#cbd3dc'
        svg += [f'<rect x="{x-ox*scale/2}" y="{y-oy*scale/2}" width="{ox*scale}" height="{oy*scale}" fill="{fill}" stroke="#243447"/>',
                f'<text x="{x}" y="{y-24}" text-anchor="middle" font-family="sans-serif" font-size="14">{name}</text>']
        for item in goods:
            if item['rack']==name:svg.append(f'<rect x="{x-4}" y="{y-4}" width="8" height="8" fill="{item["color"]}"/>')
        dx,dy=point([p[0]-config['dock_offset'],p[1]])
        svg.append(f'<circle cx="{dx}" cy="{dy}" r="3" fill="#596779"/>')
    x,y=point(config['start']);svg += [f'<circle cx="{x}" cy="{y}" r="{.16*scale}" fill="#d9e9fc" stroke="#2b6cb0"/>',
        f'<text x="{x}" y="{y+5}" text-anchor="middle" font-family="sans-serif" font-size="14">START</text>',
        f'<text x="{width/2}" y="{height-25}" text-anchor="middle" font-family="sans-serif" font-size="14">9 boxes · 119 × 85.7 × 53.8 mm · B3 empty · red 2 / green 2 / blue 1</text>','</svg>']
    (ROOT/'docs/warehouse_layout.svg').write_text('\n'.join(svg))


def create():
    config=yaml.safe_load((SHARE/'config/warehouse.yaml').read_text())
    goods=validate(config)
    source=ET.parse(SHARE/'worlds/sorting.world').getroot()
    world=source.find('world');world.set('name','warehouse')
    # Keep the proven 1 kHz contact integration for precise loaded docking.
    world.find('physics/max_step_size').text='0.001'
    world.find('physics/real_time_update_rate').text='1000'
    for m in list(world.findall('model')):
        if m.get('name')!='ground':world.remove(m)
    world.find('gui/camera/pose').text='-.5 -1.8 3.3 0 .95 .95'
    centers={**config['racks'],**{k:v['center'] for k,v in config['outputs'].items()}}
    models={}
    for name,position,size in fixture_parts(config):
        owner=next((k for k in centers if name.startswith(k+'_')),name)
        if owner not in models:
            model=child(world,'model',name=owner);child(model,'static','true');models[owner]=child(model,'link',name='frame')
        link=models[owner]
        for typ in ('collision','visual'):
            e=child(link,typ,name=name);child(e,'pose',' '.join(map(str,position))+' 0 0 0');geometry(e,size)
            if typ=='visual':
                rgb=[.45,.48,.52] if owner in config['racks'] else [.55,.58,.60]
                if owner in config['outputs']:rgb=config['colors'][config['outputs'][owner]['color']]
                if owner.startswith('wall'):rgb=[.70,.72,.74]
                material(e,rgb)
    # Labels are affixed to the front wall of each ground-level container.
    textures=SHARE/'media/materials/textures';textures.mkdir(parents=True,exist_ok=True)
    scripts=SHARE/'media/materials/scripts';scripts.mkdir(parents=True,exist_ok=True)
    font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf',48)
    materials=[]
    for name,(x,y) in centers.items():
        image=Image.new('RGB',(512,128),'white');draw=ImageDraw.Draw(image)
        draw.text((256,64),name,font=font,fill='black',anchor='mm')
        image.save(textures/f'{name}.png')
        materials.append(f'material Warehouse/{name}\n{{\n technique\n {{\n  pass\n  {{\n   lighting off\n   texture_unit\n   {{\n    texture {name}.png\n   }}\n  }}\n }}\n}}\n')
        visual=child(models[name],'visual',name=f'{name}_label')
        child(visual,'pose',f'{x-config["container_outer"][0]/2-.0001} {y} {config["container_outer"][2]/2} {math.pi/2} 0 {-math.pi/2}')
        plane=child(child(visual,'geometry'),'plane');child(plane,'normal','0 0 1');child(plane,'size','.08 .025')
        script=child(child(visual,'material'),'script');child(script,'uri','file://media/materials/scripts');child(script,'uri','file://media/materials/textures');child(script,'name',f'Warehouse/{name}')
    (scripts/'warehouse.material').write_text('\n'.join(materials))
    for item in goods:
        model=child(world,'model',name=item['id']);child(model,'pose',' '.join(map(str,item['position']))+' 0 0 0')
        link=child(model,'link',name='box'); inertial=child(link,'inertial');child(inertial,'mass',config['box_mass'])
        inertia=child(inertial,'inertia');sx,sy,sz=config['box_size'];mass=config['box_mass']
        for name,value in [('ixx',mass*(sy*sy+sz*sz)/12),('iyy',mass*(sx*sx+sz*sz)/12),('izz',mass*(sx*sx+sy*sy)/12),('ixy',0),('ixz',0),('iyz',0)]:child(inertia,name,value)
        collision=child(link,'collision',name='box');geometry(collision,config['box_size'])
        surface=child(collision,'surface');ode=child(child(surface,'friction'),'ode');child(ode,'mu',.65);child(ode,'mu2',.65)
        bounce=child(surface,'bounce');child(bounce,'restitution_coefficient',0)
        visual=child(link,'visual',name='box');geometry(visual,config['box_size']);material(visual,config['colors'][item['color']])
    ET.indent(source,space='  ')
    ET.ElementTree(source).write(SHARE/'worlds/warehouse.world',encoding='unicode',xml_declaration=True)
    resolution=.025;xmin,xmax,ymin,ymax=config['room'];origin=[xmin-.15,ymin-.15,0.]
    width=math.ceil((xmax-xmin+.30)/resolution);height=math.ceil((ymax-ymin+.30)/resolution)
    cells=bytearray([254]*(width*height));parts=fixture_parts(config)
    # Include whole fixture projections, conservatively blocking gaps under trays.
    for row in range(height):
        y=origin[1]+(height-1-row+.5)*resolution
        for col in range(width):
            x=origin[0]+(col+.5)*resolution
            occupied=not(xmin+.04<x<xmax-.04 and ymin+.04<y<ymax-.04)
            if not occupied:occupied=any(abs(x-p[0])<=d[0]/2+.0125 and abs(y-p[1])<=d[1]/2+.0125 for _,p,d in parts)
            if occupied:cells[row*width+col]=0
    (SHARE/'config/warehouse_map.pgm').write_bytes(f'P5\n{width} {height}\n255\n'.encode()+cells)
    (SHARE/'config/warehouse_map.yaml').write_text(yaml.safe_dump(dict(image='warehouse_map.pgm',resolution=resolution,origin=origin,negate=0,occupied_thresh=.65,free_thresh=.25)))
    nav=yaml.safe_load((SHARE/'config/nav2.yaml').read_text())
    for name in ('local_costmap','global_costmap'):
        p=nav[name][name]['ros__parameters'];p['robot_radius']=.16;p['resolution']=resolution;p['inflation_layer']['inflation_radius']=.23
    (SHARE/'config/warehouse_nav2.yaml').write_text(yaml.safe_dump(nav,sort_keys=False))
    layout_svg(config)
    print(f'Generated 6 racks, 3 outputs, {len(goods)} gravity-enabled cargo; map {width}x{height}')

if __name__=='__main__':create()
