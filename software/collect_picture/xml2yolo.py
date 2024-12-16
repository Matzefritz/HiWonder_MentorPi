#!/usr/bin/env python3
# encoding: utf-8
# Data:2021/10/23
import os
import sys
import glob
import yaml
import random
import argparse
import xml.etree.ElementTree as ET

# 标注之后，将数据转换成yolo格式

def split(full_list, shuffle=False):
    n_total = len(full_list)
    offset1 = int(n_total * 0.05)
    offset2 = int(n_total * 0.2)
    if n_total == 0:
        return [], full_list
    if shuffle:
        random.shuffle(full_list)

    sublist_1 = full_list[:offset1]
    sublist_2 = full_list[offset1:offset2]
    sublist_3 = full_list[offset2:]
    return sublist_1, sublist_2, sublist_3

def generate_train_and_val():
    file_list = []
    for xml_file in glob.glob(str(os.path.join(XML_PATH,'*.xml'))):
        base_name = os.path.basename(xml_file)
        jpg_file = os.path.join(JPG_PATH, base_name[:-4]+'.jpg')
        if os.path.exists(jpg_file):
            file_list.append(jpg_file)

    file_list = [''.join([x + '\n']) for x in file_list]

    test, val, train = split(file_list, True)
    with open(os.path.join(TXT_PATH, 'all.txt'), 'w') as tf:
        for t in file_list:
            tf.write(t)
    with open(os.path.join(TXT_PATH, 'train.txt'),'w') as tf:
        for t in train:
            tf.write(t)
    with open(os.path.join(TXT_PATH, 'val.txt'),'w') as tf:
        for t in val:
            tf.write(t)
    with open(os.path.join(TXT_PATH, 'test.txt'),'w') as tf:
        for t in test:
            tf.write(t)

def get_labels(path):
    with open(os.path.join(path, 'classes.names')) as f:
        class_names = f.readlines()
    class_names = [x.strip() for x in class_names]
    class_count = [0] * len(class_names)
    return class_names, class_count

#转换一个xml文件为txt
def single_xml_to_txt(xml_file, jpg_file, class_names, class_count):
    tree = ET.parse(xml_file)
    root = tree.getroot()
    #保存txt文件路径
    txt_file = os.path.join(jpg_file, os.path.basename(xml_file)[:-4] + '.txt')
    with open(txt_file, 'w') as tf:
        for member in root.findall('object'):
            #从xml获取图像的宽和高
            picture_width = int(root.find('size')[0].text)
            picture_height = int(root.find('size')[1].text)
            class_name = member[0].text
            #类名对应的index
            class_num = class_names.index(class_name)
            class_count[class_num] += 1
            box_x_min = int(member[4][0].text)  # 左上角横坐标
            box_y_min = int(member[4][1].text)  # 左上角纵坐标
            box_x_max = int(member[4][2].text)  # 右下角横坐标
            box_y_max = int(member[4][3].text)  # 右下角纵坐标
            # 转成相对位置和宽高（所有值处于0~1之间）
            x_center = (box_x_min + box_x_max) / (2 * picture_width)
            y_center = (box_y_min + box_y_max) / (2 * picture_height)
            width = (box_x_max - box_x_min) / picture_width
            height = (box_y_max - box_y_min) / picture_height
            #print(class_num, x_center, y_center, width, height)
            tf.write(str(class_num) + ' ' + str(x_center) + ' ' + str(y_center) + ' ' + str(width) + ' ' + str(
                height) + '\n')

def xml2txt(xml_path, jpg_path, class_names, class_count):
    #  转换文件夹下的所有xml文件为txt
    for xml_file in glob.glob(str(os.path.join(xml_path, '*.xml'))):
        #print(xml_file)
        try:
            single_xml_to_txt(xml_file, jpg_path, class_names, class_count)
        except Exception as e:
            pass
            #print(e, xml_file)

def create_yaml(path, data):
    file = open(path, 'w')
    yaml.dump(data, file)
    file.close()

if __name__ == '__main__':
    parse = argparse.ArgumentParser(description='data processing')
    parse.add_argument('--data', type=str, help='data path')
    parse.add_argument('--yaml', type=str, help='data.yaml path')
    args = parse.parse_args()
    BASE_PATH = args.data
    print('data:', args.data)

    XML_PATH = os.path.join(BASE_PATH, 'Annotations')
    JPG_PATH = os.path.join(BASE_PATH, 'JPEGImages')
    TXT_PATH = os.path.join(BASE_PATH, 'ImageSets')
    try:
        generate_train_and_val()
    except BaseException as e:
        print('error1:', e)
        sys.exit(0)
    
    try:   
        class_names, class_count = get_labels(BASE_PATH)
        xml2txt(XML_PATH, JPG_PATH, class_names, class_count)
    except BaseException as e:
        print('error2:', e)
        sys.exit(0)
    
    try:
        for c, n in zip(class_names, class_count):
            print(c + ':' + str(n))
    except BaseException as e:
        print('error3: ', e)

    data = {'train': os.path.join(TXT_PATH, 'train.txt'),
            'val': os.path.join(TXT_PATH, 'val.txt'),
            'nc': len(class_names),
            'names': class_names}
    try:
        create_yaml(args.yaml, data)
        print(data)
        print('finish!')
    except BaseException as e:
        print('error4:', e)

