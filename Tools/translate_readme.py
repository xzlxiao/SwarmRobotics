"""
python3 -m pip install markdown
python3 -m pip install beautifulsoup4
python3 -m pip install translate
python3 -m pip install html2markdown
"""

import markdown
from bs4 import BeautifulSoup
from translate import Translator
import html2markdown
import copy
import re
import random

def getRandomNumstr(num_len=10):
    ret = [str(int(random.random()*10)) for _ in range(num_len)]
    return ''.join(ret)

def copyMdCode(text, flags=('bash','python')):
    '''
    将markdown中的代码都复制出来，并用一串随机数字替换
    '''
    code_copy = {}
    for flag in flags:
        while True:
            pattern = re.compile('```%s(.*?)```'%flag, re.DOTALL)
            # print('```%s(.*)```'%flag)
            match = pattern.search(text)
            if match is None:
                break
            text_old = text[match.start():match.end()]
            text_new = getRandomNumstr(20)
            # pattern.sub(text_new, text)
            # print(text_old)
            text = text.replace(text_old, text_new)
            code_copy[text_new] = text_old
    return code_copy, text
    # a.string

def restoreEle(text, item_list):
    '''
    将markdown中的代码都复制出来替换回去
    '''
    for text_new, text_old in item_list.items():
        text = text.replace(text_new, text_old)
    return text

def translate_every_ele(html_tree, ele='p', except_text=None, except_replace_text=None):
    eles = html_tree.find_all(ele)
    eles_size = len(eles)
    print('---------发现%d个%s元素---------'%(eles_size, ele))
    for ind, i in enumerate(eles):
        print('[%s]: [%d|%d]'%(ele, ind+1, eles_size))
        if i.string is not None:
            if except_text is not None and i.string in except_text:
                ind = except_text.index(i.string)
                i.string = except_replace_text[ind]
            else:
                print(i.string)
                i.string = translate_to_english(i.string)

def translate_to_english(text):
    return Translator(from_lang='zh', to_lang='en').translate(text)

if __name__ == '__main__':
    file_path = 'readme_chinese.md'
    save_path = 'readme_english.md'

    text = ''
    with open(file_path, "r", encoding="utf-8") as input_file:
        text = input_file.read()
    input_file.close()

    code_back, text = copyMdCode(text, ('bash', 'python'))

    html1 = markdown.markdown(text)
    # print(html2markdown.convert(html1))
    html_tree = BeautifulSoup(html1, 'html.parser')   
    html_tree.h1.string = 'SwarmRobotics'

    except_text = ('肖镇龙', 'CONTRIBUTING.md', '中文')
    except_placed_text = ('Xiao Zhenlong(肖镇龙)', 'CONTRIBUTING.md', '中文')
    print('===============开始翻译===============')

    
    # code_data = copyEle(html_tree, 'code')
    translate_every_ele(html_tree, 'p', except_text=except_text, except_replace_text=except_placed_text)
    translate_every_ele(html_tree, 'h2', except_text=except_text, except_replace_text=except_placed_text)
    translate_every_ele(html_tree, 'li', except_text=except_text, except_replace_text=except_placed_text)
    translate_every_ele(html_tree, 'a', except_text=except_text, except_replace_text=except_placed_text)
    # restoreEle(html_tree, code_data, 'code')
    # html_tree.
    
    html2 = str(html_tree)
    

    md_text = html2markdown.convert(html2)
    md_text = restoreEle(md_text, code_back)
    
    # print(markdown)
    with open(save_path, "w", encoding="utf-8", errors="xmlcharrefreplace") as output_file:
        output_file.write(md_text)
