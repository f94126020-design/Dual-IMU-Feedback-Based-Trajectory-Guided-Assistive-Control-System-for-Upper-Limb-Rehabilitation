from pathlib import Path

from docx import Document
from docx.enum.section import WD_SECTION
from docx.enum.table import WD_CELL_VERTICAL_ALIGNMENT, WD_TABLE_ALIGNMENT
from docx.enum.text import WD_ALIGN_PARAGRAPH, WD_BREAK, WD_LINE_SPACING
from docx.oxml import OxmlElement
from docx.oxml.ns import qn
from docx.shared import Cm, Inches, Pt, RGBColor


ROOT = Path(__file__).resolve().parent
OUT = ROOT / "基於雙IMU回授之上肢復健按需輔助控制系統_正式計畫書_第六版_專題進度對齊修訂.docx"

BLUE = "2E74B5"
DARK_BLUE = "1F4D78"
NAVY = "0B2545"
MUTED = "5E6B75"
LIGHT = "F4F6F9"
GRAY = "D9E1E8"
WHITE = "FFFFFF"
BLACK = "000000"


def set_cell_shading(cell, fill):
    tc_pr = cell._tc.get_or_add_tcPr()
    shd = tc_pr.find(qn("w:shd"))
    if shd is None:
        shd = OxmlElement("w:shd")
        tc_pr.append(shd)
    shd.set(qn("w:fill"), fill)


def set_cell_margins(cell, top=80, start=120, bottom=80, end=120):
    tc = cell._tc
    tc_pr = tc.get_or_add_tcPr()
    tc_mar = tc_pr.first_child_found_in("w:tcMar")
    if tc_mar is None:
        tc_mar = OxmlElement("w:tcMar")
        tc_pr.append(tc_mar)
    for m, v in (("top", top), ("start", start), ("bottom", bottom), ("end", end)):
        node = tc_mar.find(qn(f"w:{m}"))
        if node is None:
            node = OxmlElement(f"w:{m}")
            tc_mar.append(node)
        node.set(qn("w:w"), str(v))
        node.set(qn("w:type"), "dxa")


def set_repeat_table_header(row):
    tr_pr = row._tr.get_or_add_trPr()
    tbl_header = OxmlElement("w:tblHeader")
    tbl_header.set(qn("w:val"), "true")
    tr_pr.append(tbl_header)


def set_row_cant_split(row):
    tr_pr = row._tr.get_or_add_trPr()
    cant_split = tr_pr.find(qn("w:cantSplit"))
    if cant_split is None:
        cant_split = OxmlElement("w:cantSplit")
        tr_pr.append(cant_split)


def set_table_geometry(table, widths_dxa):
    table.alignment = WD_TABLE_ALIGNMENT.CENTER
    table.autofit = False
    tbl_pr = table._tbl.tblPr
    tbl_w = tbl_pr.find(qn("w:tblW"))
    if tbl_w is None:
        tbl_w = OxmlElement("w:tblW")
        tbl_pr.append(tbl_w)
    tbl_w.set(qn("w:w"), str(sum(widths_dxa)))
    tbl_w.set(qn("w:type"), "dxa")
    tbl_ind = tbl_pr.find(qn("w:tblInd"))
    if tbl_ind is None:
        tbl_ind = OxmlElement("w:tblInd")
        tbl_pr.append(tbl_ind)
    tbl_ind.set(qn("w:w"), "120")
    tbl_ind.set(qn("w:type"), "dxa")
    grid = table._tbl.tblGrid
    for child in list(grid):
        grid.remove(child)
    for width in widths_dxa:
        col = OxmlElement("w:gridCol")
        col.set(qn("w:w"), str(width))
        grid.append(col)
    for row in table.rows:
        set_row_cant_split(row)
        for idx, cell in enumerate(row.cells):
            cell.width = Inches(widths_dxa[idx] / 1440)
            tc_pr = cell._tc.get_or_add_tcPr()
            tc_w = tc_pr.find(qn("w:tcW"))
            if tc_w is None:
                tc_w = OxmlElement("w:tcW")
                tc_pr.append(tc_w)
            tc_w.set(qn("w:w"), str(widths_dxa[idx]))
            tc_w.set(qn("w:type"), "dxa")
            set_cell_margins(cell)
            cell.vertical_alignment = WD_CELL_VERTICAL_ALIGNMENT.CENTER


def set_repeat_keep(paragraph, keep_with_next=False, keep_together=False):
    p_pr = paragraph._p.get_or_add_pPr()
    if keep_with_next:
        node = p_pr.find(qn("w:keepNext"))
        if node is None:
            node = OxmlElement("w:keepNext")
            p_pr.append(node)
    if keep_together:
        node = p_pr.find(qn("w:keepLines"))
        if node is None:
            node = OxmlElement("w:keepLines")
            p_pr.append(node)


def set_run_font(run, size=None, bold=None, italic=None, color=BLACK):
    run.font.name = "Calibri"
    run._element.get_or_add_rPr().rFonts.set(qn("w:ascii"), "Calibri")
    run._element.get_or_add_rPr().rFonts.set(qn("w:hAnsi"), "Calibri")
    run._element.get_or_add_rPr().rFonts.set(qn("w:eastAsia"), "Microsoft JhengHei")
    if size is not None:
        run.font.size = Pt(size)
    if bold is not None:
        run.bold = bold
    if italic is not None:
        run.italic = italic
    run.font.color.rgb = RGBColor.from_string(color)


def configure_styles(doc):
    normal = doc.styles["Normal"]
    normal.font.name = "Calibri"
    normal._element.rPr.rFonts.set(qn("w:ascii"), "Calibri")
    normal._element.rPr.rFonts.set(qn("w:hAnsi"), "Calibri")
    normal._element.rPr.rFonts.set(qn("w:eastAsia"), "Microsoft JhengHei")
    normal.font.size = Pt(11)
    normal.paragraph_format.alignment = WD_ALIGN_PARAGRAPH.JUSTIFY
    normal.paragraph_format.space_before = Pt(0)
    normal.paragraph_format.space_after = Pt(8)
    normal.paragraph_format.line_spacing = 1.333

    for style_name, size, color, before, after in (
        ("Heading 1", 16, BLUE, 18, 10),
        ("Heading 2", 13, BLUE, 12, 6),
        ("Heading 3", 12, DARK_BLUE, 8, 4),
    ):
        style = doc.styles[style_name]
        style.font.name = "Calibri"
        style._element.rPr.rFonts.set(qn("w:ascii"), "Calibri")
        style._element.rPr.rFonts.set(qn("w:hAnsi"), "Calibri")
        style._element.rPr.rFonts.set(qn("w:eastAsia"), "Microsoft JhengHei")
        style.font.size = Pt(size)
        style.font.bold = True
        style.font.color.rgb = RGBColor.from_string(color)
        style.paragraph_format.space_before = Pt(before)
        style.paragraph_format.space_after = Pt(after)
        style.paragraph_format.keep_with_next = True

    for style_name in ("List Bullet", "List Number"):
        style = doc.styles[style_name]
        style.font.name = "Calibri"
        style._element.rPr.rFonts.set(qn("w:eastAsia"), "Microsoft JhengHei")
        style.font.size = Pt(11)
        style.paragraph_format.left_indent = Inches(0.375)
        style.paragraph_format.first_line_indent = Inches(-0.194)
        style.paragraph_format.space_after = Pt(4)
        style.paragraph_format.line_spacing = 1.208

    caption = doc.styles["Caption"]
    caption.font.name = "Calibri"
    caption._element.rPr.rFonts.set(qn("w:eastAsia"), "Microsoft JhengHei")
    caption.font.size = Pt(9.5)
    caption.font.color.rgb = RGBColor.from_string(MUTED)
    caption.paragraph_format.alignment = WD_ALIGN_PARAGRAPH.CENTER
    caption.paragraph_format.space_before = Pt(4)
    caption.paragraph_format.space_after = Pt(8)


def add_page_number(paragraph):
    paragraph.alignment = WD_ALIGN_PARAGRAPH.RIGHT
    run = paragraph.add_run("第 ")
    set_run_font(run, size=9, color=MUTED)
    fld = OxmlElement("w:fldSimple")
    fld.set(qn("w:instr"), "PAGE")
    paragraph._p.append(fld)
    run = paragraph.add_run(" 頁")
    set_run_font(run, size=9, color=MUTED)


def configure_page(doc):
    doc.settings.odd_and_even_pages_header_footer = False
    for section in doc.sections:
        section.page_width = Inches(8.5)
        section.page_height = Inches(11)
        section.top_margin = Inches(1)
        section.bottom_margin = Inches(1)
        section.left_margin = Inches(1)
        section.right_margin = Inches(1)
        section.header_distance = Inches(0.492)
        section.footer_distance = Inches(0.492)
        section.different_first_page_header_footer = False
        hp = section.header.paragraphs[0]
        hp.clear()
        hp.text = "國立成功大學生物醫學工程學系畢業專題｜上肢動作量測與按需輔助控制"
        hp.alignment = WD_ALIGN_PARAGRAPH.RIGHT
        for run in hp.runs:
            set_run_font(run, size=8.5, color=MUTED)
        fp = section.footer.paragraphs[0]
        fp.clear()
        add_page_number(fp)


def add_heading(doc, text, level=1):
    p = doc.add_paragraph(text, style=f"Heading {level}")
    set_repeat_keep(p, keep_with_next=True)
    return p


def add_body(doc, text, bold_lead=None):
    p = doc.add_paragraph()
    if bold_lead and text.startswith(bold_lead):
        r1 = p.add_run(bold_lead)
        set_run_font(r1, bold=True)
        r2 = p.add_run(text[len(bold_lead):])
        set_run_font(r2)
    else:
        r = p.add_run(text)
        set_run_font(r)
    set_repeat_keep(p, keep_together=True)
    return p


def add_bullet(doc, text, numbered=False):
    p = doc.add_paragraph(style="List Number" if numbered else "List Bullet")
    r = p.add_run(text)
    set_run_font(r)
    return p


def add_numbered_items(doc, items):
    numbering = doc.part.numbering_part.element
    abstract_ids = [int(x.get(qn("w:abstractNumId"))) for x in numbering.findall(qn("w:abstractNum"))]
    num_ids = [int(x.get(qn("w:numId"))) for x in numbering.findall(qn("w:num"))]
    abstract_id = max(abstract_ids, default=0) + 1
    num_id = max(num_ids, default=0) + 1

    abstract = OxmlElement("w:abstractNum")
    abstract.set(qn("w:abstractNumId"), str(abstract_id))
    multi = OxmlElement("w:multiLevelType")
    multi.set(qn("w:val"), "singleLevel")
    abstract.append(multi)
    lvl = OxmlElement("w:lvl")
    lvl.set(qn("w:ilvl"), "0")
    start = OxmlElement("w:start")
    start.set(qn("w:val"), "1")
    num_fmt = OxmlElement("w:numFmt")
    num_fmt.set(qn("w:val"), "decimal")
    lvl_text = OxmlElement("w:lvlText")
    lvl_text.set(qn("w:val"), "%1.")
    suff = OxmlElement("w:suff")
    suff.set(qn("w:val"), "tab")
    p_pr = OxmlElement("w:pPr")
    tabs = OxmlElement("w:tabs")
    tab = OxmlElement("w:tab")
    tab.set(qn("w:val"), "num")
    tab.set(qn("w:pos"), "540")
    tabs.append(tab)
    ind = OxmlElement("w:ind")
    ind.set(qn("w:left"), "540")
    ind.set(qn("w:hanging"), "280")
    p_pr.extend([tabs, ind])
    lvl.extend([start, num_fmt, lvl_text, suff, p_pr])
    abstract.append(lvl)
    numbering.append(abstract)

    num = OxmlElement("w:num")
    num.set(qn("w:numId"), str(num_id))
    abstract_ref = OxmlElement("w:abstractNumId")
    abstract_ref.set(qn("w:val"), str(abstract_id))
    num.append(abstract_ref)
    lvl_override = OxmlElement("w:lvlOverride")
    lvl_override.set(qn("w:ilvl"), "0")
    start_override = OxmlElement("w:startOverride")
    start_override.set(qn("w:val"), "1")
    lvl_override.append(start_override)
    num.append(lvl_override)
    numbering.append(num)

    for text in items:
        p = doc.add_paragraph()
        p.paragraph_format.space_after = Pt(4)
        p.paragraph_format.line_spacing = 1.208
        ppr = p._p.get_or_add_pPr()
        numpr = OxmlElement("w:numPr")
        ilvl = OxmlElement("w:ilvl")
        ilvl.set(qn("w:val"), "0")
        nid = OxmlElement("w:numId")
        nid.set(qn("w:val"), str(num_id))
        numpr.extend([ilvl, nid])
        ppr.append(numpr)
        r = p.add_run(text)
        set_run_font(r)


def add_equation(doc, text):
    p = doc.add_paragraph()
    p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    p.paragraph_format.space_before = Pt(4)
    p.paragraph_format.space_after = Pt(8)
    r = p.add_run(text)
    set_run_font(r, size=11, italic=True, color=NAVY)
    return p


def add_table(doc, headers, rows, widths_dxa):
    table = doc.add_table(rows=1, cols=len(headers))
    table.style = "Table Grid"
    header = table.rows[0]
    set_repeat_table_header(header)
    for idx, value in enumerate(headers):
        cell = header.cells[idx]
        cell.text = ""
        set_cell_shading(cell, LIGHT)
        p = cell.paragraphs[0]
        p.alignment = WD_ALIGN_PARAGRAPH.CENTER
        p.paragraph_format.space_after = Pt(0)
        r = p.add_run(value)
        set_run_font(r, size=9.5, bold=True, color=NAVY)
    for row_data in rows:
        row = table.add_row()
        for idx, value in enumerate(row_data):
            cell = row.cells[idx]
            cell.text = ""
            p = cell.paragraphs[0]
            p.paragraph_format.space_after = Pt(0)
            p.paragraph_format.line_spacing = 1.15
            r = p.add_run(str(value))
            set_run_font(r, size=9.2)
    set_table_geometry(table, widths_dxa)
    doc.add_paragraph().paragraph_format.space_after = Pt(2)
    return table


def add_status_callout(doc, label, text):
    table = doc.add_table(rows=1, cols=1)
    table.style = "Table Grid"
    cell = table.cell(0, 0)
    set_cell_shading(cell, LIGHT)
    p = cell.paragraphs[0]
    p.paragraph_format.space_after = Pt(0)
    r = p.add_run(f"{label}：")
    set_run_font(r, bold=True, color=NAVY)
    r = p.add_run(text)
    set_run_font(r)
    set_table_geometry(table, [9360])
    doc.add_paragraph().paragraph_format.space_after = Pt(2)


def add_cover(doc):
    for _ in range(3):
        doc.add_paragraph()
    p = doc.add_paragraph()
    p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    r = p.add_run("畢業專題研究計畫書")
    set_run_font(r, size=14, bold=True, color=MUTED)
    p.paragraph_format.space_after = Pt(18)

    p = doc.add_paragraph()
    p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    p.paragraph_format.space_after = Pt(8)
    r = p.add_run("基於雙 IMU 回授與目標軌跡導引之\n上肢復健按需輔助控制系統設計")
    set_run_font(r, size=23, bold=True, color=NAVY)

    p = doc.add_paragraph()
    p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    p.paragraph_format.space_after = Pt(36)
    r = p.add_run("Design of a Dual-IMU Feedback-Based, Trajectory-Guided\nAssist-as-Needed Control System for Upper-Limb Rehabilitation")
    set_run_font(r, size=12, italic=True, color=MUTED)

    add_table(doc, ["計畫資訊", "內容"], [
        ("學校／系所", "國立成功大學／生物醫學工程學系"),
        ("組別／實驗室", "醫學電子組／生醫訊號與神經工程實驗室"),
        ("學生", "[姓名、學號待填]"),
        ("指導教授", "陳家進 特聘教授"),
        ("計畫版本", "正式計畫書第五版（完整研究設計審查修訂）"),
        ("日期", "2026 年 8 月"),
    ], [2300, 7060])
    doc.add_page_break()


def build():
    doc = Document()
    configure_styles(doc)
    configure_page(doc)
    add_cover(doc)

    add_heading(doc, "摘要", 1)
    add_body(doc, "全球超過 24 億人可能受益於復健服務，其中約 8,600 萬人因中風而有復健需求 [1], [19]。對仍保有部分上肢動作能力者而言，固定訓練範圍無法反映個別與跨工作階段差異；輔助不足可能限制任務完成，輔助過度則可能掩蓋自主出力。2025 年納入 54 項研究、共 2,744 名中風者的統合分析顯示，部分輔助裝置的上肢能力結果優於劑量相當的傳統復健，但研究對輔助量的定義及校準方式仍不一致 [20]。本研究由醫學電子與生醫訊號處理觀點出發，建立雙慣性量測單元（Inertial Measurement Unit, IMU）上肢運動量測、指定時間自由活動式個人化活動範圍（Range of Motion, ROM）參考參數、目標軌跡及多模式輔助控制工程雛形。Raspberry Pi 負責姿態融合、ROM 百分位估測、目標軌跡、持續柔和助力、表現式 AAN、PID、線性自抗擾控制（LADRC）、迭代學習控制（ILC）、頻域系統識別與資料紀錄；ESP32 負責三路馬達命令授權、300 ms 通訊逾時、輸出漸變、反轉歸零及鎖定式軟體停止。最新開發紀錄顯示，上述軟體模組已部署至樹莓派並完成 48/48 項自動化測試，但測試證據限於軟體、合成模型與未致動檢查；尚未以真實馬達自動執行 Chirp，亦未完成閉迴路效能、實際輔助力或人體安全驗證。研究將依『資料與部署閘門—感測與 ROM—固定測試架系統識別—控制器比較—故障反應』順序進行非臨床驗證。本研究的貢獻是整合『個別化參考參數—可辨識系統動態—即時回授—可解釋／可學習介入—可追溯紀錄』，並明確區分軟體通過、台架性能及人體／臨床證據。")
    add_body(doc, "關鍵字：上肢運動量測、雙 IMU、個人化活動範圍、Assist-as-Needed、系統識別、LADRC、迭代學習控制、嵌入式故障反應")

    add_heading(doc, "一、研究背景與動機", 1)
    add_heading(doc, "1.1 復健需求與上肢功能障礙", 2)
    add_body(doc, "人口老化、慢性疾病與神經系統損傷使復健需求持續增加。Cieza 等人依全球疾病負擔研究估計，2019 年約有 24.1 億人罹患可能受益於復健介入的健康狀況，較 1990 年增加約 63% [1]；世界衛生組織指出，其中約 8,600 萬人因中風及其相關功能問題具有復健需求，且部分地區仍存在顯著的服務缺口 [19]。中風後上肢肌力、協調與動作控制受損，可能限制進食、穿衣及其他日常活動。相關復健原則重視高重複性、任務導向及自主動作參與，因此評估系統時除動作完成與軌跡誤差外，亦應考量外部輔助的介入程度。")

    add_heading(doc, "1.2 為什麼需要輔助復健", 2)
    add_body(doc, "部分患者保有運動意圖與殘餘動作能力，卻可能因肌力不足、協調異常或疲勞而無法完成預定軌跡。完全自主訓練可能限制成功動作的重複量，完全被動帶動則不要求自主動作貢獻；主動輔助訓練位於兩者之間，由使用者先產生動作，外部裝置再補足未完成部分。Kahn 等人的隨機先導研究顯示，機器人主動輔助可用於中風後上肢伸取訓練 [6]。因此，本計畫在工程層將目標定義為：維持任務追蹤表現，同時限制非必要的馬達命令介入。")
    add_body(doc, "機器人輔助復健具有動作重複性、參數可調性與資料可記錄性，但裝置效益仍取決於控制模式與介入劑量。2025 年統合分析納入 54 項隨機研究與 2,744 名中風者，整體上肢能力改善雖達統計顯著，效果量偏小且未於追蹤期維持。其次群組分析顯示，輔助程度與結果具有顯著關聯（p＝0.0046）；部分輔助組相較傳統復健之標準化平均差為 0.37（95% CI 0.19–0.55），全程輔助組則估計不精確且未達顯著 [20]。由於全程輔助研究數量有限且各研究對輔助量的定義不一致，本計畫僅將此結果作為控制策略設計依據，不作因果或臨床療效推論。")

    add_heading(doc, "1.3 固定輔助的限制與按需輔助", 2)
    add_body(doc, "當外部輔助量不足時，使用者可能無法完成目標動作；當輔助量過高時，位置誤差雖可降低，卻可能出現自主出力下降的 motor slacking 現象。Wolbrecht 等人據此將最低必要輔助列為復健機器人控制的重要原則 [3]；Pehlivan 等人則將任務誤差與機器人輔助量共同納入最小 AAN 控制設計 [21]。因此，AAN 的工程目標並非單純最小化軌跡誤差，而是在預先定義的任務容許範圍內降低不必要的控制投入。")
    add_body(doc, "應特別區分『軌跡追蹤表現』與『生理層級主動參與』。僅依位置誤差調節輸出時，自主出力降低可能由致動器補償，故低誤差不能直接視為高參與度 [9]。本研究未配置互動力、力矩或肌電訊號感測，因此採用『表現式 AAN』之保守定位：以死區抑制小幅誤差，並於誤差持續超過設定時間後加入積分補償；評估指標包含追蹤誤差、控制介入時間與控制輸出量。所得結果僅用以判定控制器是否依運動表現調整介入，不作神經肌肉參與程度推論。")

    add_heading(doc, "1.4 為什麼採用雙 IMU 回授", 2)
    add_body(doc, "按需輔助需要即時判斷關節是否跟上目標軌跡。光學動作捕捉精度高，但設備、空間與遮蔽限制較大；機構編碼器則只反映馬達或機構位置，未必等同人體肢段姿態。IMU 具有低成本、體積小、易穿戴及不依賴外部攝影空間等特點。將兩顆 IMU 分別配置於上臂與前臂，可由相鄰肢段相對姿態估測肘關節屈伸，降低單一前臂感測器把上臂整體移動誤認為肘屈伸的問題 [10]-[13]。")
    add_body(doc, "IMU 並非沒有誤差。陀螺儀積分會漂移，加速度計在動態加速度下會失真，感測器與肢段軸線不一致也會造成系統性偏差。相關研究顯示，功能性校正、感測器融合及運動學限制可提升肘角估測表現 [10]-[13]。因此，本研究將雙 IMU 定位為低成本即時回授方案，並規劃以量角器或編碼器作參考真值，實際驗證固定角度誤差、重複性與動態追蹤，而不直接援用他人最佳誤差作為本系統成果。")

    add_heading(doc, "1.5 醫學電子與生醫訊號研究定位", 2)
    add_body(doc, "本研究所量測之 IMU 資料屬穿戴式運動學訊號，而非肌電、腦電等電生理訊號；其醫學電子價值在於整合感測器校正、時序取樣、雜訊抑制、姿態融合、個人化 ROM 參數、系統識別與閉迴路決策。一般控制 CSV 已可保存時間戳記、處理後角度、目標速度、誤差、回授／前饋／合併輸出、三路期望／實際 PWM 與狀態欄位；頻域模組另輸出 raw_frequency_data.csv、frequency_analysis.json 與 frequency_analysis.png。惟現行紀錄仍未確認逐筆保存六軸原始加速度與角速度，也未整合外部參考角度的同步欄位。因此，正式資料收集前仍須新增或確認 ax、ay、az、gx、gy、gz、校正與參數版本、參考角度及同步事件；若未完成同步，只能報告取樣穩定性與控制內部性能，不得宣稱動態量測準確度或相位差。")

    add_heading(doc, "1.6 研究動機總結", 2)
    add_status_callout(doc, "核心研究命題", "先以雙 IMU 建立工作階段相對的個人化 ROM 參考參數，再以固定測試架辨識『有效關節 PWM→IMU 關節角度』動態，據以比較 PID／持續柔和助力／LADRC 與 ILC 組合控制；在維持追蹤與安全閘門的前提下，量化誤差、命令投入、重複軌跡學習及故障反應。")

    add_heading(doc, "二、相關文獻回顧", 1)
    add_heading(doc, "2.1 上肢機器人復健與主動輔助訓練", 2)
    add_body(doc, "上肢機器人可依與人體連接方式分為外骨骼型與末端牽引型。外骨骼型可直接對應人體關節，但需處理關節軸線對位、重量與安全；末端牽引型機構較簡化，但關節運動由末端約束間接形成 [4], [14]。Karadeniz 等人所開發的四自由度肩肘外骨骼展示了機構強度、安全活動範圍及 PID 位置控制的重要性 [14]，但其規模也說明完整多自由度外骨骼不適合作為本階段唯一成功條件。")
    add_body(doc, "系統性回顧與臨床試驗顯示，機器人訓練可作為中風上肢復健工具，但效果受到裝置、訓練內容、劑量與受試者程度影響 [2], [7]。主動輔助模式適合仍有部分動作能力但無法獨立完成任務者；然而輔助模式必須依功能程度選擇。Jeon 等人比較阻力與主動輔助機器訓練時發現，中度障礙受試者在阻力組的部分結果較佳 [8]，顯示輔助越多不等於效果越好。故本研究不宣稱 AAN 對所有患者皆最佳，而是把它視為可依動作表現調節輔助的工程策略。")

    add_heading(doc, "2.2 Assist-as-Needed 控制策略", 2)
    add_body(doc, "AAN 方法包含阻抗／導納控制、虛擬通道、模型式能力估測、EMG 觸發與資料驅動調節等 [3]-[5]。Zhang 等人以虛擬通道讓使用者在允許區域內自由運動，偏離後才產生恢復力 [15]；另一項表現式控制研究則以追蹤誤差切換零互動、AAN 與限制區域模式 [16]。這些方法提供本研究兩項設計原則：第一，需設定不介入區域；第二，輔助強度不宜僅由單一瞬間誤差決定，應考量誤差持續時間與控制狀態。")

    add_heading(doc, "2.3 IMU 上肢角度估測", 2)
    add_body(doc, "Fang 等人彙整 52 篇將上肢 IMU 資料轉換為關節角度的研究，指出 Euler 角分解、Kalman 類濾波、功能性關節軸校正及運動學限制皆被廣泛使用，且不同關節與自由度的誤差差異明顯 [10]。El-Gohary 與 McNames 以兩顆 IMU、運動學模型及 Unscented Kalman Filter 追蹤肩肘角度 [11]；Müller 等人則以自校正方法處理感測器對位，肘屈伸相對光學參考的 RMS 誤差為 2.7° [12]。Morrow 等人的研究亦顯示商用 IMU 的肘屈伸誤差會隨量測幅度改變，結果具有特定操作協定依賴性 [13]。因此，本研究必須以自身安裝與動作條件建立校正及驗證。")

    add_heading(doc, "2.4 IMU 活動範圍量測與個人化參數", 2)
    add_body(doc, "IMU 用於 ROM 量測具有便攜與連續記錄優勢，但有效度取決於關節、動作平面、安裝與校正程序。Costa 等人以 29 名無症狀成人比較慣性感測器與量角器，肘部主動 ROM 具良好一致性，但該結果來自特定商用系統與標準化量測流程，不能直接移植為本系統準確度 [22]。Kaszyński 等人以 15 名無症狀成人驗證肩部 ROM，結果雖呈中度至優良相關，但一致性界限仍可達數十度，顯示『相關性高』不等於個別量測誤差小 [23]。因此，本研究同時報告偏差、MAE、重複性及一致性界限，不只報告相關係數。")
    add_body(doc, "現行程式將量得 ROM 的 90% 映射為訓練目標；此比例是可調的工程預設值，不是文獻證實的患者安全劑量。於任何人體使用前，必須增加治療人員可設定的比例、角度硬上／下限與停止條件。本研究中的肩部訊號係單顆上臂 IMU 相對初始重力方向的抬舉角，可能混入軀幹與肩胛代償，故僅稱『肩部任務之上臂抬舉參考參數』，不稱解剖學盂肱關節 ROM。")

    add_heading(doc, "2.5 PID 追蹤、安全限制與本研究取捨", 2)
    add_body(doc, "PID 容易實作、參數意義清楚，適合建立基礎閉迴路與比較 P／PI／PID 對誤差、震盪與穩定時間的影響。復健裝置中的 PID 研究通常以超越量、上升時間、穩定時間及軌跡誤差評估 [14], [17]。然而，導數項會放大量測雜訊，積分項在輸出飽和時可能產生 windup；人體接觸系統亦不能僅由上位軟體控制器保證安全。因此，本研究使用 deadband、積分限幅、輸出飽和及漸變輸出，並由 ESP32 下位節點執行上電禁能、通訊逾時、反轉歸零與鎖定式軟體停止狀態。")

    add_heading(doc, "2.6 系統識別、LADRC 與 ILC", 2)
    add_body(doc, "本系統未配置馬達編碼器，因此研究輸入／輸出模型定義為『經輸出整形與三路分配後的有效關節 PWM→雙 IMU 估測關節角度』，其動態同時包含馬達、繩索、機構、負載與感測濾波，而非馬達軸模型。頻域模組使用 Chirp、PRBS 或單頻正弦激振，估測 H1 頻率響應、Bode 圖、coherence、頻寬、相位延遲及 ARX 模型；coherence 用於辨識哪些頻帶的輸入／輸出關聯較可信。自動掃頻只允許無人體的固定測試架。")
    add_body(doc, "ADRC 以擴張狀態觀測器把未知模型項與外擾合併為總擾動並在線估測；本研究採二階 LADRC，另以 ESO 子步積分降低 Linux 偶發較大取樣間隔造成數值不穩定的風險 [24]。ILC 則利用重複任務前一周期的誤差更新下一周期前饋，適合有限時間、重複執行的軌跡追蹤 [25]；上肢復健相關研究亦曾將 ILC 用於重複動作控制 [26]。本系統採 200 個相位格點、P-type 更新、Q-filter、忘卻因子、單周期更新上限與 learned PWM 上限，且只允許完整、未飽和的正弦周期學習。這些模組目前屬已實作待實機驗證的方法，不先宣稱優於 PID。")

    add_heading(doc, "三、研究缺口、問題與假設", 1)
    add_heading(doc, "3.1 研究缺口", 2)
    for text in (
        "單一姿態感測器難以區分上臂整體移動與肘關節相對屈伸；雙 IMU 方案則需處理校正、濾波與安裝誤差。",
        "傳統位置控制追求低誤差，卻可能以持續馬達代償換取追蹤表現，無法證明使用者主動參與。",
        "高階 AAN 常依賴力矩感測器、EMG、精確動力學模型或高階運算，低成本雛形需要較簡潔、可解釋且可驗證的替代策略。",
        "不同使用者及不同訓練日的可動範圍可能不同；若以固定角度範圍產生目標軌跡，可能超出當下能力或無法形成足夠挑戰，因此需要可重複的個人化 ROM 參考參數。",
        "只比較 RMSE 無法證明『按需』；必須同時評估啟動時機、介入時間與命令量。又因目前沒有力／力矩感測器，PWM 只能代表命令層介入，不能當成實際機械輔助力。",
        "動態角度準確度、相位差與控制延遲需要共同時間基準；現行紀錄尚未納入外部參考角度，若未先完成同步就無法形成有效的動態效度結論。",
        "人體接觸式致動需要多層風險控制；僅在 Raspberry Pi 程式中限制 PWM，無法涵蓋通訊中斷、控制程式失效、驅動器故障與過電流等情境。",
        "現行系統無馬達編碼器、力／力矩或張力感測器；因此只能由 PWM 與 IMU 建立等效輸入／輸出模型，不能宣稱精確放線、繩索張力、關節扭矩或實際輔助力控制。",
        "LADRC 與 ILC 雖已完成軟體及合成模型測試，仍缺固定測試架上的頻域辨識、參數調整及跨控制器實機比較；這是目前最關鍵的研究證據缺口。",
    ):
        add_bullet(doc, text)

    add_heading(doc, "3.2 問題定義", 2)
    add_body(doc, "本研究探討在未使用光學動作捕捉、馬達編碼器、關節力／力矩感測器或完整人體動力學模型的條件下，能否以雙 IMU 建立可驗證的單平面肘角訊號，以指定時間自由活動與穩健百分位衍生工作階段相對 ROM 參考參數，再由固定測試架系統識別支持持續柔和助力、表現式 AAN、LADRC 與 ILC 組合控制的比較。研究結論限於非臨床台架與演算法層級：肘部可討論相對角度準確度；肩部僅討論上臂抬舉參考；PWM 僅代表命令量；任何控制結果均不推論使用者意圖、實際輔助力或療效。")

    add_heading(doc, "3.3 研究問題", 2)
    questions = [
        "雙 IMU 在固定角度與動態屈伸條件下，能否穩定估測單自由度肘關節角度？",
        "個人化 ROM 流程能否穩定判定肘部主軸與方向，並產生具重複性的肘屈伸與肩部任務之上臂抬舉參考參數？",
        "固定測試架的有效關節 PWM→IMU 關節角度動態能否由 FRF 與 ARX 模型重複辨識，並界定可信頻帶、延遲與模型適用範圍？",
        "PID、持續柔和助力、LADRC 與前饋＋LADRC 能否在相同台架條件下穩定追蹤週期性目標軌跡？",
        "誤差門檻式 AAN 是否能在維持可接受追蹤性能時降低命令層介入量；持續柔和助力是否能避免病患跟上軌跡時致動器完全停止？",
        "ILC＋PID 與 ILC＋LADRC 在有效完整周期中是否呈現跨周期 RMSE 或回授修正量下降，且中斷／飽和周期不污染學習？",
        "ESP32 與 IMU watchdog 能否在未 ARM、通訊逾時、I²C 失聯、鎖定式軟體停止及方向反轉等情況下產生預期的故障反應？",
    ]
    for idx, q in enumerate(questions, start=1):
        add_bullet(doc, f"RQ{idx}：{q}")

    add_heading(doc, "3.4 研究邏輯追溯矩陣", 2)
    add_table(doc, ["命題", "主要終點", "對應實驗", "預先判定原則"], [
        ("RQ1／H1", "靜態 bias、MAE、SD；同步動態 RMSE", "實驗一、二", "靜態 MAE ≤ 5°、穩定段 SD ≤ 2°；動態資料須先通過同步品質門檻"),
        ("RQ2／H2", "肘部軸向／方向一致率、ROM 重複差與參考差", "實驗三", "5 次軸向／方向一致率 100%；肘 ROM 平均絕對參考差 ≤ 5°、重複 SD ≤ 5°"),
        ("RQ3／H3", "FRF、coherence、頻寬、延遲、ARX 驗證", "實驗四", "先以 coherence 與獨立回合驗證界定可用模型；未通過不得據以調整 LADRC"),
        ("RQ4／H4", "RMSE、phase lag、jerk、PWM RMS／變化量", "實驗五", "無持續震盪、限幅失控或安全中止；以鎖定參數呈現模式取捨"),
        ("RQ5／H5", "累積絕對 PWM、作用時間比例與 RMSE", "實驗六", "AAN 與持續柔和助力分別對照全程 PID；命令量不解釋為力"),
        ("RQ6／H6", "逐周期 RMSE、改善率、learned PWM、有效周期率", "實驗七", "只納入完整未飽和周期；學習輸出不超過預設上限"),
        ("RQ7／H7", "狀態轉移、IMU age／errors 與輸出歸零時間", "實驗八", "各故障事件 5/5 進入預定狀態；通訊 timeout 至歸零 ≤ 350 ms"),
    ], [1150, 2850, 1850, 3510])
    add_body(doc, "上述為先導型工程驗收門檻，用於讓命題可被否證，不是臨床最小重要差異。正式測試前將把門檻、控制參數、負載、參考工具及排除規則寫入版本化測試表；開始正式資料收集後不得依結果調整。")

    add_heading(doc, "四、研究目的、範圍與成果", 1)
    add_heading(doc, "4.1 研究目的", 2)
    objectives = (
        "建立雙 IMU 讀取、啟動零點校正、互補濾波、相對肢段角度與後端平滑流程。",
        "建立肘屈曲、肘伸展與肩部前舉／側舉任務之上臂抬舉參考流程，並將結果映射為個別工作階段的訓練參數。",
        "建立固定、階躍與由最小角度平順起始的正弦目標軌跡，以及 PID、持續柔和助力與表現式 AAN 控制模式。",
        "建立安全的 Chirp／PRBS／單頻正弦系統識別流程，估測 FRF、coherence、頻寬、延遲與 ARX 模型。",
        "建立二階 LADRC 與 P-type ILC，並比較 PID、LADRC、ILC＋PID 與 ILC＋LADRC 的非臨床台架表現。",
        "整合 Raspberry Pi、ESP32、BTS7960、三顆直流減速馬達、Web GUI 與 CSV 紀錄。",
        "建立感測、ROM、控制、AAN 命令層介入與故障反應之量化實驗，形成可重複分析之時序資料與研究圖表。",
    )
    for idx, objective in enumerate(objectives, start=1):
        add_bullet(doc, f"O{idx}：{objective}")

    add_heading(doc, "4.2 研究範圍", 2)
    add_table(doc, ["面向", "本研究納入", "本階段不宣稱／不納入"], [
        ("動作", "肘關節屈伸為主要閉迴路驗證；ROM 模組納入肘屈／伸及肩部任務的上臂抬舉參考", "解剖學肩關節角度或完整三維上肢運動重建"),
        ("感測", "雙 IMU 相對角度、角速度、角加速度、jerk、取樣間隔與訊號品質", "醫療級動作捕捉精度"),
        ("控制", "PID、持續柔和助力、誤差與時間式 AAN、LADRC、ILC＋PID、ILC＋LADRC", "LQR、MPC、AI 控制及真正力／扭矩控制"),
        ("致動", "對應肘屈曲、肘伸展與肩部輔助方向之三路致動通道", "宣稱重現個別肌肉生物力學或完成臨床級外骨骼"),
        ("驗證", "固定治具、機構、ROM 參數重複性與非臨床工程測試", "未經倫理審查之人體資料、病患療效或醫療級 ROM 評估"),
        ("故障反應", "軟體限幅與 ESP32 下位狀態機", "醫療器材安全認證或完整硬體安全層"),
    ], [1400, 4300, 3660])

    add_heading(doc, "五、系統架構與研究方法", 1)
    add_heading(doc, "5.1 整體架構", 2)
    add_equation(doc, "雙 IMU／ROM → 目標軌跡 → 系統識別／PID／AAN／LADRC／ILC → 輸出整形與三路分配 → ESP32 → 馬達／繩索／關節")
    add_body(doc, "Raspberry Pi 為上位控制器，負責三個 TCA9548A 通道的 MPU6050 讀取，其中上臂與前臂兩顆 IMU 構成肘角核心回授；現行程式另保留第三感測通道作擴充。Web GUI 用於校正、選擇目標關節與軌跡、調整控制器參數、啟閉致動輸出、下達鎖定式軟體停止命令及 CSV 紀錄。ESP32 為下位 PWM 執行與通訊故障反應節點，不負責研究層 AAN 決策，亦不取代實體硬體保護。")
    add_table(doc, ["元件／工具", "研究角色", "正式紀錄要求"], [
        ("Raspberry Pi＋Web GUI", "感測融合、ROM、目標、控制與資料紀錄", "程式雜湊、參數、live／dry-run 狀態"),
        ("TCA9548A＋MPU6050", "多通道慣性量測；兩顆構成肘角核心", "通道、感測器位置、方向與校正紀錄"),
        ("ESP32", "PWM 執行、命令解析與故障反應", "韌體版本、baud rate、狀態回覆"),
        ("BTS7960＋三顆直流減速馬達", "肘屈、肘伸及肩部方向致動", "接線、電源限制、方向、溫升與測試負載"),
        ("角度參考工具＋固定治具", "靜態／動態參考與重複姿勢", "型號、解析度／精度規格、同步方式"),
    ], [2350, 3800, 3210])

    add_heading(doc, "5.2 雙 IMU 姿態估測", 2)
    add_body(doc, "MPU6050 設定為 ±2 g 加速度範圍、±250°/s 角速度範圍，感測器內部在啟用數位低通時以 200 Hz 取樣；此值不等同 Python 控制迴圈的實際更新率。程式啟動後進行靜態校正，取得各通道 roll／pitch 基準角與陀螺儀偏移。每次更新以互補濾波融合陀螺儀短期動態與加速度計重力方向，再扣除基準角；正式報告以時間戳實測有效更新率、取樣間隔分布與資料缺口，不宣稱固定 200 Hz 控制頻率。")
    add_body(doc, "正式試驗以標記治具固定上臂與前臂 IMU，使其選定軸與肢段長軸關係可重現；每回合記錄位置與方向並重新執行零點／陀螺儀偏移校正。若感測器鬆動、位置改變或校正失敗，該回合依排除規則處理，不以事後旋轉或選軸修正結果。")
    add_equation(doc, "θ_elbow,raw(t) = |θ_forearm(t) − θ_upper-arm(t)|")
    add_equation(doc, "θ_fused(t) = α[θ_fused(t−1) + ω(t)Δt] + (1−α)θ_acc(t)")
    add_body(doc, "現行預設 α = 0.96，肘角再以一維 Kalman 平滑。上述差角法只適用於感測器軸線一致且以單平面肘屈伸為主的條件，因此實驗必須固定安裝方向、執行零點校正，並以參考角度工具驗證；若未來進行三維動作，需改為相對旋轉矩陣或四元數方法。")

    add_heading(doc, "5.3 個人化 ROM 參考參數", 2)
    add_body(doc, "ROM 模組以四個指定動作建立工作階段相對參考：肘伸直到最大屈曲、肘約 90° 到最大伸展、自然下垂到最大前舉，以及自然下垂到最大側舉。每個動作可設定 5–30 s，預設 10 s；期間由使用者依自身能力自由往返，不要求固定節奏、保持區段或指定次數。系統以所有有效樣本的第 5／95 百分位估測穩健最小／最大角度，降低單次 IMU 尖峰直接成為 ROM 端點的風險。程式另依肘部資料判定主要旋轉軸與方向；肩部結果保存上臂相對初始重力方向的抬舉角與方向參考，不視為盂肱關節角。")
    add_body(doc, "現行程式將量得範圍的 90% 映射為訓練上下限並套用至 GUI。此 90% 僅為工程預設，尚未具患者安全或治療劑量證據；本研究只驗證映射是否正確與可重複。任何人體使用前的軟體需求包括：比例可調、治療人員核准、角度硬限位、疼痛／不適停止條件及可回復的設定紀錄。本計畫將結果稱為『個人化 ROM 參考參數』，不用於診斷、取代臨床量角器或宣稱醫療級測角效度。")

    add_heading(doc, "5.4 目標軌跡", 2)
    add_body(doc, "系統提供階躍與正弦軌跡。階躍模式於低角度與高角度間切換，用於評估上升時間、超越量、穩定時間與穩態誤差；正弦模式用於模擬連續屈伸，評估 RMSE、相位延遲與平順度。")
    add_equation(doc, "θ_target(t) = θ_center + A sin(2πt/T)")
    add_body(doc, "固定 30° 至 90°、週期 5 s 僅作台架控制比較的標準化起始條件；個人化流程則使用 ROM 映射範圍，兩者不得混為同一實驗。正式測試前先以低 PWM、無人體負載逐級確認方向與限位，並在測試表記錄所用範圍、週期與負載。")

    add_heading(doc, "5.5 分層式按需輔助策略", 2)
    add_body(doc, "目前 GUI 保留純 PID、落後才介入的表現式 AAN，以及『持續柔和助力』。後者採 u=u_ff+u_PID，其中速度前饋依目標速度與方向使致動器在使用者能跟上軌跡時仍同步收／放線，PID 僅在追蹤誤差增加時補足；導數誤差改為『目標速度−量測速度』，避免把正常跟隨動作當成阻尼抵抗。最新部署預設為 2°／1° 介入／解除門檻、assist delay 0.10 s、前饋增益 0.35 PWM/(°/s) 與最低移動前饋 8 PWM；這些均為待固定測試架調整的工程初值。")
    add_table(doc, ["狀態", "判斷", "控制行為"], [
        ("允許區內", "|e| ≤ 1°", "誤差回授接近零；持續柔和模式於目標移動時仍保留速度前饋"),
        ("瞬間偏差", "|e| > 2° 且持續時間 < 0.10 s", "P／D 立即修正，I 項尚未啟用"),
        ("持續落後", "|e| > 2° 且持續時間 ≥ 0.10 s", "啟用 I 項並在限幅內增加輔助"),
        ("遲滯保持／解除", "介入後 1° < |e| ≤ 2°／|e| ≤ 1°", "前者維持狀態，後者解除誤差介入並釋放積分"),
        ("危險／失效", "軟體停止、未啟用、通訊逾時或命令非法", "輸出立即歸零並鎖定或回到 IDLE"),
    ], [1500, 3300, 4560])
    add_body(doc, "本方法屬低成本、誤差表現式 AAN，不等同直接量測人體主動力矩。正式結論只能說明其能否依軌跡落後調節介入；若要證明生理層級主動參與，未來仍需 EMG、力矩或互動力感測。")

    add_heading(doc, "5.6 PID、LADRC、ILC 與輸出限制", 2)
    add_equation(doc, "u(t) = Kp e_c(t) + Ki∫e_c(t)dt + Kd de(t)/dt")
    add_body(doc, "其中 e_c 為 deadband 處理後的控制誤差。積分值限制於 ±Imax，原始控制輸出限制於 ±umax；程式已實作輸出飽和時的條件積分 anti-windup。導數項採目標速度與量測速度之差，並以時間常數 τ＝0.08 s 的一階低通抑制雜訊。LADRC 目前提供純 LADRC 與前饋＋LADRC，保守初值為控制器頻寬 ωc=2 rad/s、觀測器頻寬 ωo=8 rad/s、輸入增益 b0=1；參數必須由實機識別結果修訂。ILC 提供 ILC＋PID 與 ILC＋LADRC，只在完整、未飽和的正弦周期更新 200 點 learned feedforward；急停、中斷、資料非有限值或 anti-windup 啟動的周期均不學習。所有控制模式最終仍通過相同輸出飽和、斜率限制、換向停頓與 ESP32 安全層。")

    add_heading(doc, "5.7 拮抗方向致動通道配置", 2)
    add_body(doc, "現行系統設置肘屈曲、肘伸展與肩部輔助三路致動通道；其命名只表示作用方向，不宣稱重現個別肌肉力學。肘屈曲命令使屈曲側收線並使伸展側釋放，肘伸展則相反；拮抗側釋放比例預設為 0.8。持續柔和助力版本將命令低通時間常數縮短為 0.08 s，收線／釋放斜率提高為 240／300 PWM/s，反轉停頓縮短為 80 ms；上述設定旨在降低遲鈍感，仍須以台架確認振動、超越與安全餘裕。三路馬達已可實際輸出控制，但因無力／力矩、張力與電流校正，PWM 指標不可換算為輔助力、機械功或能量。")

    add_heading(doc, "5.8 ESP32 下位執行與故障反應節點", 2)
    add_table(doc, ["故障反應功能", "現行設計", "驗證方式"], [
        ("上電禁能", "啟動為 IDLE，驅動器 REN／LEN 為 LOW", "重新上電觀察輸出與狀態"),
        ("命令授權", "PWM 前必須收到 ARM", "未 ARM 發送 PWM，應回覆 NOT_ARMED"),
        ("通訊逾時", "300 ms 未收到有效 PWM 即 TIMEOUT_STOP", "停止送命令並量測停機反應"),
        ("輸出漸變", "每 10 ms 最多改變 5 PWM", "記錄 target 與 applied PWM"),
        ("受控反轉", "反轉前先降至零，至少經過一個零輸出 tick", "正負命令切換測試"),
        ("鎖定式軟體停止", "ESTOP 命令使輸出歸零，需 CLEAR 後重新 ARM", "故障注入與狀態轉移測試"),
        ("嚴格解析", "非法格式與超範圍 PWM 不更新 timeout", "邊界與錯誤命令測試"),
    ], [1700, 5000, 2660])
    add_body(doc, "上述設計屬軟體與通訊層之故障反應機制，不能取代實體急停、硬體斷電、電流限制、溫度監測與機械限位。現階段編碼器功能亦於 ESP32 韌體中停用，故本計畫不將其描述為完整安全系統或醫療器材安全設計；在任何人體接觸測試前，必須先補齊硬體保護並完成風險評估。")

    add_heading(doc, "5.9 GUI 與資料紀錄", 2)
    add_body(doc, "Web GUI 可執行自由活動式 ROM、固定／階躍／正弦軌跡、純 PID、落後介入、持續柔和助力、LADRC、前饋＋LADRC、ILC＋PID、ILC＋LADRC，以及 Chirp／PRBS／單頻正弦系統識別。畫面可顯示 IMU age／累積錯誤、前饋與回授輸出、LADRC 估測狀態、ILC 周期／覆蓋率／RMSE 改善率及 learned PWM。控制 CSV 已增加目標速度、PID 回授、前饋與合併輸出；頻域分析另輸出 CSV、JSON 與 PNG。『原始角度』仍不等同 MPU6050 六軸原始資料；正式研究前須補齊六軸欄位、外部參考角度、同步事件與版本資訊。")

    add_heading(doc, "5.10 實作基線與版本控制", 2)
    add_body(doc, "最新開發 task 記載：整合版已部署於樹莓派，Flask／systemd 服務正常，GUI 過期的 DRY RUN 文字已修正，系統在驗證結束時保持未初始化、未 ARM 與零 PWM；完整備份為 /home/user/Desktop/imu_PIDcontrol/backup_complete_controls_20260811。惟目前工作資料夾不必然等同該遠端部署版本，因此正式實驗前仍須由樹莓派匯出唯一部署基線，保存原始碼雜湊、live 設定、ESP32 韌體、接線、感測器位置、馬達方向、電源限制及全部控制參數。缺少任一版本證據的回合只列開發紀錄。")
    add_table(doc, ["正式測試前閘門", "通過條件", "未通過處理"], [
        ("資料完整性", "六軸原始 IMU、參考角度、同步事件、參數與版本均可寫入", "不得進行動態效度與正式比較"),
        ("部署一致性", "GUI、Pi 與 ESP32 版本及 live 輸出設定可追溯", "只列為開發測試"),
        ("方向與限位", "三路低 PWM 正／反方向、零輸出與機械範圍逐項確認", "停機修正，不以軟體符號補救未知接線"),
        ("故障反應", "STOP、軟體鎖定停止與 300 ms timeout 在無人體條件通過", "禁止進入閉迴路負載測試"),
    ], [2100, 4660, 2600])

    add_heading(doc, "六、實驗設計與資料分析", 1)
    add_heading(doc, "6.1 實驗原則", 2)
    add_body(doc, "本研究採模組化、分階段的非臨床工程驗證，不將人體或患者作為正式研究樣本。順序為『資料與部署閘門 → 感測器與 ROM → 三路致動／故障反應 → 固定測試架系統識別 → PID／持續柔和助力／LADRC 比較 → ILC 跨周期學習』；前一階段未通過時不得進入下一階段。所有致動先在固定治具、限流與無人體負載條件執行。")
    add_body(doc, "每一正式條件完成 5 次獨立回合，回合是統計與圖表的實驗單位，單一回合內的高頻樣本不得當成獨立樣本。5 次重複用於先導型工程重複性與失敗模式篩查，不足以支持臨床推論或穩健的族群統計。參數調整回合標記為 pilot，不納入正式比較；正式階段將控制模式順序隨機化或平衡化，回合間保留固定冷卻／重置時間。")

    add_heading(doc, "6.2 實驗一：雙 IMU 靜態角度準確度", 2)
    add_table(doc, ["項目", "設計"], [
        ("目的", "驗證零點校正後固定肘角的準確度、精密度與漂移"),
        ("角度", "0°、30°、45°、60°、75°、90°"),
        ("參考值", "正式測試前指定一項具規格資料的電子量角器或校驗後編碼器；不得於結果階段更換"),
        ("重複", "每角度 5 回；每回重新歸零並穩定記錄 10 s"),
        ("指標", "bias、MAE、RMSE、標準差、最大誤差、每分鐘漂移"),
    ], [1800, 7560])

    add_heading(doc, "6.3 實驗二：動態角度與濾波比較", 2)
    add_body(doc, "以固定機構產生週期 8 s、5 s 與 3 s 的肘屈伸，每一速度 5 回，並以共同時間戳或可辨識同步事件取得參考角度。比較差角原始值、互補濾波與後端 Kalman 結果，計算 bias、MAE、RMSE、phase lag、峰值角速度差、有效更新率、取樣間隔平均值／SD／95 百分位及資料缺口比例；另以 Bland-Altman 圖檢視偏差與一致性界限。同步誤差必須小於一個控制更新週期；若參考訊號未寫入同一資料流或無法驗證同步，本實驗只報告取樣穩定性與內部濾波比較，不報告動態準確度、phase lag 或 Bland-Altman。")

    add_heading(doc, "6.4 實驗三：個人化 ROM 參數驗證", 2)
    add_body(doc, "以可重複調整的關節治具模擬肘屈曲、肘伸展及兩種上臂抬舉方向，固定感測器位置後完整執行四階段流程 5 次。記錄肘部主軸、方向符號、最小／最大角度、ROM、90% 映射值，以及肩部任務的上臂抬舉角與方向參考。肘部與參考角度比較 bias、MAE、重複 SD 及最大回合差；肩部只檢查方向分類與參數重複性。5 次工程回合不計算 ICC，也不把治具結果稱為人體 ROM 效度。另以已知輸入確認 90% 映射公式正確；此驗證不代表 90% 適合患者。")

    add_heading(doc, "6.5 實驗四：固定測試架系統識別與頻域分析", 2)
    add_body(doc, "先以固定測試架、無人體負載及低 PWM 完成單頻正弦，再依安全閘門執行 Chirp 與 PRBS。每一輸入條件至少 5 回，輸入採經輸出整形與三路分配後的有效關節 PWM，輸出為 IMU 肘角。分析 H1 FRF、Bode magnitude／phase、coherence、共振頻率、−3 dB 頻寬、等效相位延遲、實際取樣率、timing jitter 與 ARX 模型；ARX 自動搜尋 0–5 個樣本輸入延遲，並以獨立回合驗證。超出安全角度、IMU／ESP32 異常或時間完成均立即 STOP。首次建議僅以肘關節、PWM 8、0.1–1 Hz、30–60 s 測試；在取得可靠 FRF 前不得連接人體或啟用高頻 LADRC／ILC。")

    add_heading(doc, "6.6 實驗五：PID、持續柔和助力與 LADRC 追蹤比較", 2)
    add_body(doc, "使用台架標準範圍 30°–90°，分別測試 8 s、5 s 與 3 s 正弦週期；先完成無負載，再加入一項事前記錄的固定負載。比較 PID、持續柔和助力、LADRC 與前饋＋LADRC，每條件 5 回且順序平衡化。LADRC 的 b0、ωc、ωo 由實驗四可用頻帶與輸入增益調整，調參回合標記為 pilot。主要指標為 RMSE、MAE、phase lag、峰值誤差、jerk、PWM RMS、PWM total variation、飽和比例及安全中止數；不得只報告最佳曲線。")

    add_heading(doc, "6.7 實驗六：AAN 命令層介入特性", 2)
    add_table(doc, ["比較模式", "說明", "目的"], [
        ("無輔助", "只量測，不輸出馬達", "取得誤差注入基準，不代表自主動作"),
        ("全程 PID", "deadband_on／off＝0，I 項無延遲", "代表持續位置追蹤控制"),
        ("落後介入 AAN", "2° 啟動／1° 解除＋0.10 s 後積分", "檢驗命令層介入是否下降"),
        ("持續柔和助力", "目標速度前饋＋gated PID", "檢驗跟上軌跡時仍同步致動且不過度抵抗"),
    ], [1600, 4300, 3460])
    add_body(doc, "本實驗先以版本化合成序列驗證 2°／1° 遲滯、0.10 s 積分延遲、目標速度−量測速度導數項與速度前饋；再於相同台架、目標、負載與輸出上限下比較無輔助、全程 PID、落後介入 AAN 與持續柔和助力，每模式 5 回且順序平衡化。比較 RMSE、控制作用時間比例、平均／累積絕對 PWM、峰值 PWM、首次介入／解除延遲，以及目標移動且誤差位於死區內時的非零前饋比例。原先『AAN 累積 PWM 降低至少 20% 且 RMSE 增加不超過 20%』保留為暫定先導門檻，但持續柔和助力的目的不同，不套用同一減量門檻。")

    add_heading(doc, "6.8 實驗七：ILC 跨周期學習", 2)
    add_body(doc, "在固定測試架、正弦軌跡與相同初始條件下，比較 PID、LADRC、ILC＋PID 與 ILC＋LADRC。每個正式序列至少含 10 個完整周期並重複 5 回；報告逐周期 RMSE、相對第一周期改善率、learned PWM peak、回授修正量、有效／無效周期數與覆蓋率。另刻意注入停止、飽和與不完整周期，確認該周期不更新學習。ILC 結果只代表特定重複台架任務的誤差學習，不推論患者學習或神經恢復。")

    add_heading(doc, "6.9 實驗八：下位節點與 IMU 故障反應", 2)
    add_body(doc, "測試未 ARM 命令、正常 ARM、300 ms 逾時、STOP、鎖定式軟體停止、CLEAR、非法 PWM、超範圍 PWM 及正負方向反轉；每事件重複 5 次，記錄主機命令時間、ESP32 狀態回覆、target PWM、applied PWM 與歸零時間。STOP、鎖定式停止與 timeout 依韌體設計會略過 ramp 並將驅動禁能；反轉則先漸變至零並至少保留一個 10 ms 零輸出 tick。所有測試均在無人體負載與低電源限制下執行，結果不等同醫療器材安全驗證。")

    add_body(doc, "另測試 IMU 心跳逾時與 reader 卡死情境：瞬時 I²C 例外應先停馬達；連續 2 s 無成功 IMU 資料時 watchdog 結束服務，由 systemd 於 2 s 後重啟。重啟後必須維持未初始化、未 ARM、零 PWM。")

    add_heading(doc, "6.10 統計與訊號分析原則", 2)
    add_body(doc, "連續指標以每回合數值、平均值、SD、中位數與範圍呈現；控制模式比較同時報告逐回合配對差與百分比差。由於每條件只有 5 個工程回合，本研究不以 p 值或 ICC 支持一般化推論，也不把回合內的數千筆樣本當作樣本數。Bland-Altman 只用於同步且成對的角度資料，相關係數不作為一致性的唯一證據。分析程式、濾波、時間窗與門檻於正式測試前鎖定。")
    add_body(doc, "預定排除條件為：參考同步失敗、任一資料缺口 > 0.5 s、該回合缺失比例 > 1%、非計畫性 STOP／timeout、機械碰限或軟硬體版本不符。所有被排除回合仍保存並列出原因；排除後可依原條件補做一次，但不得刪除不利結果而不報告。")

    add_heading(doc, "6.11 指標與公式", 2)
    add_equation(doc, "MAE = (1/N) Σ |θ_ref,i − θ_meas,i|")
    add_equation(doc, "RMSE = √[(1/N) Σ (θ_target,i − θ_measured,i)²]")
    add_equation(doc, "控制作用時間比例 = N(|PWM| > 0) / N_total × 100%")
    add_equation(doc, "平均絕對命令量 = (1/N) Σ |PWM_i|")
    add_equation(doc, "累積絕對命令量 = Σ |PWM_i| Δt_i")
    add_body(doc, "phase lag 以目標與量測訊號的互相關或同頻相位差估計；jerk 由角加速度的一階差分取得，但需固定取樣頻率並先處理雜訊。所有結果呈現平均值、標準差與各次試驗散布，不只報告單一最佳曲線。")

    add_heading(doc, "6.12 預定成功標準", 2)
    add_table(doc, ["驗證面向", "暫定工程驗收標準", "備註"], [
        ("靜態肘角", "MAE ≤ 5°，單次穩定段 SD ≤ 2°", "參考工具須於正式測試前固定"),
        ("動態資料品質", "參考角度同步可驗證；缺失比例 ≤ 1%；無 > 0.5 s 資料缺口", "未通過則不報告動態效度"),
        ("ROM 參考參數", "5 次肘軸向／方向一致率 100%；肘 ROM 參考 MAE ≤ 5°、重複 SD ≤ 5°", "肩部只驗證上臂抬舉參數重複性"),
        ("階躍追蹤", "無持續震盪；超越量與穩定時間可重複量測", "不先承諾不切實際的 0% overshoot"),
        ("週期追蹤", "RMSE ≤ 10° 且曲線無失控", "依負載與週期分層報告"),
        ("表現式 AAN", "相較全程 PID，累積絕對 PWM 降低 ≥ 20%，且 RMSE 相對增加 ≤ 20%", "PWM 僅為命令代理量"),
        ("系統識別", "安全事件 0 次；可界定 coherence 合格頻帶並以獨立回合驗證 ARX", "門檻於 pilot 後、正式資料前鎖定"),
        ("LADRC／ILC", "無失控或安全中止；ILC 有效周期的 learned PWM 不超限", "效能優劣由完整重複資料決定，不預設一定優於 PID"),
        ("故障反應", "各事件 5/5 進入預定狀態；timeout 至輸出歸零 ≤ 350 ms", "不等同醫療器材安全驗證"),
    ], [1800, 5200, 2360])

    add_heading(doc, "6.13 資料管理與可重現性", 2)
    add_body(doc, "每回合以匿名測試代碼命名，保存原始 CSV、唯讀備份、參數 JSON／表單、程式雜湊值、韌體版本、接線圖、參考工具資訊及排除紀錄。原始檔不覆寫；處理後資料輸出至獨立資料夾。圖表必須能由固定分析程式自原始資料重新產生。正式結果至少呈現全部有效回合、排除流程、失敗回合數與使用的軟硬體版本。若日後取得人體資料，另依倫理核准內容處理身分資料、保存期限與存取權限。")

    add_heading(doc, "七、目前進度", 1)
    add_table(doc, ["模組", "目前狀態", "證據／待辦"], [
        ("IMU 與 GUI", "已部署穩定性修正", "14-byte block read、TCA 切換／讀取鎖、reader 生命週期、IMU age／errors 與 watchdog 已實作；5 s 實機通訊壓力測試每通道 364 次、共 728 次、0 錯誤；仍待角度參考與長時間測試"),
        ("個人化 ROM", "自由活動式流程已部署", "每項 5–30 s、預設 10 s，自由往返並以 5%／95% 百分位估測端點；90% 映射保留，待治具參考與重複性驗證"),
        ("目標軌跡", "已實作", "正弦由最小角度且速度 0 平順起始，另有固定與階躍；待固定測試協定"),
        ("基礎／輔助控制", "已部署", "純 PID、落後介入 AAN、持續柔和助力；GUI 已移除 P／PI，最新預設為 2°／1°、0.10 s delay 與速度前饋；27/27 自動化測試通過，待實機量化"),
        ("頻域系統識別", "功能已部署、實機辨識未做", "Chirp／PRBS／單頻、H1 FRF、Bode、coherence、ARX 與 0–5 樣本延遲搜尋已完成；尚未以真實馬達自動執行 Chirp"),
        ("LADRC／ILC", "功能已部署、待實機驗證", "LADRC、前饋＋LADRC、ILC＋PID、ILC＋LADRC、ESO 子步積分與學習保護已完成；整合版 48/48 軟體／合成測試通過，不代表實機控制效能"),
        ("三路致動分配", "已可實際輸出控制", "肘屈／肘伸／肩部三路可動；正式研究前須從樹莓派匯出並鎖定唯一 live 部署基線"),
        ("ESP32 下位節點", "韌體與 API 已整合", "ARM、timeout、ramp、反轉歸零、STOP／ESTOP；自動測試已涵蓋 API 與狀態轉移，待正式計時與台架重複試驗"),
        ("硬體可靠性", "已恢復基本運轉，驗證未完成", "三路馬達可輸出控制；仍須完成重複啟停、反轉、溫升、長時間運轉及供電保護測試"),
        ("資料紀錄", "處理後資料已可記錄", "已有角度、控制器與 PWM；正式實驗前補六軸原始 IMU、參考角度、同步事件及版本欄位"),
        ("性能實驗", "尚未完成", "需依第六章完成重複試驗與圖表"),
    ], [1800, 2200, 5360])

    add_heading(doc, "八、預期成果與創新貢獻", 1)
    add_heading(doc, "8.1 預期成果", 2)
    for text in (
        "可即時顯示與記錄上臂、前臂及肘關節角度的雙 IMU 量測模組。",
        "可由四項指定時間自由活動與穩健百分位產生肘屈／伸及肩部任務上臂抬舉參考，並映射為當次訓練目標範圍。",
        "可設定目標軌跡、PID／AAN／持續柔和助力／LADRC／ILC、系統識別及故障反應參數的 Web GUI。",
        "可輸出 FRF、Bode、coherence、ARX 與輸入延遲等固定測試架系統識別結果。",
        "可比較即時回授、速度前饋、總擾動估測與跨周期學習等不同補償機制。",
        "具上電禁能、命令授權、通訊逾時、PWM 漸變、反轉保護及鎖定式軟體停止的 ESP32 節點。",
        "包含感測準確度、ROM 重複性、控制性能、AAN 命令層介入及故障狀態的完整數據與圖表。",
    ):
        add_bullet(doc, text)

    add_heading(doc, "8.2 創新與貢獻定位", 2)
    add_body(doc, "本專題之貢獻定位為可驗證的醫學電子與控制系統整合，而非宣稱雙 IMU、ROM、PID、ADRC 或 ILC 單一演算法首創。第一，以 14-byte block read、互斥通道操作、reader 生命週期與 watchdog 提升雙 IMU 量測鏈可靠性；第二，把指定時間自由活動與 5%／95% 穩健百分位轉成工作階段相對 ROM 參考；第三，在無編碼器條件下明確建立『有效關節 PWM→IMU 角度』的系統識別對象；第四，於同一平台整合落後介入、速度前饋持續助力、LADRC 及 ILC，並以共同安全層與共同指標比較；第五，以完整周期、飽和與中斷條件限制 ILC 學習，避免把失敗周期寫入前饋；第六，以 ESP32、IMU watchdog、版本鎖定與資料閘門形成可追溯故障反應鏈。其價值是把個人化參數、訊號可靠性、動態辨識、即時抗擾與重複任務學習串成可驗證平台，同時不把 PWM 誤稱為力，也不把軟體測試誤稱為人體或臨床成果。")
    add_heading(doc, "8.3 預期價值與影響", 2)
    add_body(doc, "在完成本階段命令層驗證後，後續可加入表面肌電（sEMG）辨識與互動力感測，建立動作表現以外的意圖或參與指標；相關 sEMG 控制研究可作為延伸設計參考 [18]，但不屬於本階段成果。")
    add_table(doc, ["層面", "本研究可帶來的價值", "合理邊界"], [
        ("量測方法", "建立穿戴式慣性訊號至肘關節相對角度的可重複處理與驗證流程。", "不視為醫療級動作擷取或臨床評估工具。"),
        ("個人化參數", "將當次肘部可達範圍及上臂抬舉任務結果轉成目標參考。", "90% 為工程預設；未有臨床安全劑量證據。"),
        ("控制研究", "同時保存角度、誤差、命令量及故障事件，使控制介入可追溯與比較。", "未量測互動力或 EMG，不能解釋實際輔助力或自主參與。"),
        ("醫學電子系統", "以嵌入式感測、分層控制與下位故障反應節點建立可重複測試的閉迴路平台。", "硬體保護、耐久度與人體工學仍須完成。"),
        ("後續發展", "提供加入 EMG、力矩感測、個人能力估測與自適應控制的共同平台 [18]。", "本階段不把追蹤誤差直接等同主動參與。"),
    ], [1700, 5000, 2660])

    add_heading(doc, "九、工作時程與里程碑", 1)
    add_table(doc, ["階段", "工作內容", "可交付成果"], [
        ("第 1 週", "由樹莓派匯出並鎖定整合部署版本；補齊六軸、參考角度、同步事件與參數版本欄位", "部署雜湊、資料字典與同步試錄"),
        ("第 2 週", "完成馬達啟停、反轉、溫升、IMU watchdog 與 ESP32 故障反應台架測試", "故障時間線與可靠性紀錄"),
        ("第 3 週", "完成雙 IMU 靜態／動態角度及自由活動式 ROM 治具重複性驗證", "角度誤差、同步與 ROM 一致性表"),
        ("第 4 週", "以固定測試架執行低 PWM 單頻、Chirp／PRBS 與 ARX 驗證", "Bode、coherence、頻寬、延遲與模型報告"),
        ("第 5 週", "依識別結果調整並比較 PID、持續柔和助力、LADRC 與前饋＋LADRC", "RMSE、phase lag、jerk、PWM 與抗擾比較"),
        ("第 6 週", "完成落後介入 AAN 與持續柔和助力比較；執行 ILC＋PID／LADRC 跨周期測試", "命令投入、追蹤與逐周期收斂圖"),
        ("第 7 週", "重複試驗、故障注入、補測、整理限制與結論", "全部回合、排除表與可重現資料"),
        ("第 8 週", "完成海報、3–4 張 Pitch Talk 投影片與口頭演練", "正式海報、簡報與 3 分鐘講稿"),
    ], [1500, 5000, 2860])

    add_heading(doc, "十、風險管理與備案", 1)
    add_table(doc, ["風險", "影響", "處理與備案"], [
        ("ESP32／驅動器過熱或燒毀", "硬體停擺與安全疑慮", "先用限流電源；確認共地、邏輯隔離、續流與反電動勢；低 PWM 單馬達逐步測試"),
        ("馬達反轉突波", "重啟、損壞或機構衝擊", "ESP32 反轉先歸零、PWM ramp；硬體端增加保護元件並驗證"),
        ("IMU 漂移／安裝誤差", "角度與追蹤指標失真", "固定治具、每回合校正、縮短試驗、感測融合、參考真值比較"),
        ("I²C 傳輸阻塞", "GUI 無回應且量測中斷", "14-byte block read、通道鎖、舊 reader 結束等待、2 s watchdog、systemd 2 s 後重啟；重啟後保持未 ARM"),
        ("參考訊號未同步", "動態 MAE、相位差與 Bland-Altman 無效", "共同時間戳或同步事件；未通過則只報靜態與內部性能"),
        ("live／dry-run 版本不一致", "無法追溯實際控制版本", "正式測試前鎖定原始碼雜湊、設定與韌體；不符者只列開發資料"),
        ("90% ROM 映射不適合人體", "可能超出安全或治療需求", "本階段只驗證工程映射；人體前加入比例可調、硬限位與專業人員核准"),
        ("導數雜訊", "PWM 抖動", "導數低通、限制 Kd、固定取樣時間、比較不用 D 的結果"),
        ("積分 windup", "超越與安全風險", "integral clamp、deadband 內衰減、輸出飽和時條件積分"),
        ("LADRC 觀測器受排程延遲影響", "估測發散或輸出突變", "ESO 子步積分、限制觀測器頻寬、依實測 dt 與辨識頻寬調整"),
        ("ILC 學習污染", "錯誤前饋跨周期累積", "飽和、anti-windup、中斷、不完整或覆蓋不足周期不更新；限制單次更新與 learned PWM"),
        ("機構未完成", "無法完成穿戴式整合驗證", "改用固定式肘關節桿件或負載台，聚焦感測、閉迴路控制與故障反應"),
        ("無法進行人體測試", "不能呈現真實主動出力", "以可重複擾動、配重或軟體誤差注入驗證 AAN，不宣稱臨床參與"),
        ("時程不足", "資料不完整", "優先完成肘關節、雙 IMU、個人化 ROM、表現式 AAN 與故障反應；EMG、AI 與肩部閉迴路性能移至未來工作"),
    ], [2000, 2500, 4860])

    add_heading(doc, "十一、研究倫理、安全與限制", 1)
    add_body(doc, "本研究現階段只進行固定治具與無人體的工程驗證，不招募健康受試者或患者，不進行治療，也不宣稱臨床療效或醫療器材資格。個人化 ROM 的設計目的，是作為未來患者訓練目標設定的候選參考參數；在臨床效度、安全性與倫理審查完成前，不得用於醫療決策。若後續任何人穿戴或接觸致動機構，均視為新增研究階段，須依成大及合作醫療機構規定確認人體研究倫理、知情同意、風險告知、疼痛監測、停止條件與緊急處置。")
    add_body(doc, "主要限制包括：肘角差分假設感測器軸線一致與近似單平面運動；MPU6050 無磁力計，無法可靠提供長時間絕對 yaw；單顆上臂 IMU 的抬舉角不能分離軀幹、肩胛與盂肱運動；所有輔助模式均未量測肌力、互動力或意圖；PWM 不是實際力或能量；無馬達編碼器，故系統識別只描述有效 PWM 至 IMU 角度的等效動態；LADRC／ILC 尚未完成真實馬達驗證；遠端部署版仍須匯出鎖定；現階段樣本為工程回合而非人群。成功標準屬先導工程驗收，不是臨床最小重要差異。")

    add_heading(doc, "十二、結論", 1)
    add_body(doc, "本計畫提出雙 IMU 相對肘角回授、自由活動式個人化 ROM、有效關節 PWM→IMU 角度系統識別，以及 PID／AAN／持續柔和助力／LADRC／ILC 的整合架構。最新進度已完成軟體部署、IMU 通訊穩定性修正與 48/48 自動化測試，但尚未以真實馬達執行自動 Chirp，也未完成閉迴路與控制器比較。後續研究將先鎖定部署與資料基線，再依固定測試架完成辨識、控制與故障反應驗證。肩部結果只解釋為上臂抬舉參考，PWM 只解釋為命令量，軟體測試不外推為人體或臨床成效。")

    add_heading(doc, "參考文獻", 1)
    references = [
        "[1] A. Cieza, K. Causey, K. Kamenov, S. W. Hanson, S. Chatterji, and T. Vos, ‘Global estimates of the need for rehabilitation based on the Global Burden of Disease study 2019,’ The Lancet, vol. 396, no. 10267, pp. 2006-2017, 2020. doi: 10.1016/S0140-6736(20)32340-0.",
        "[2] X. Yang, X. Shi, X. Xue, and Z. Deng, ‘Efficacy of robot-assisted training on rehabilitation of upper limb function in patients with stroke,’ Archives of Physical Medicine and Rehabilitation, vol. 104, no. 9, pp. 1498-1513, 2023. doi: 10.1016/j.apmr.2023.02.004.",
        "[3] E. T. Wolbrecht, V. Chan, D. J. Reinkensmeyer, and J. E. Bobrow, ‘Optimizing compliant, model-based robotic assistance to promote neurorehabilitation,’ IEEE Transactions on Neural Systems and Rehabilitation Engineering, vol. 16, no. 3, pp. 286-297, 2008. doi: 10.1109/TNSRE.2008.918389.",
        "[4] L. Marchal-Crespo and D. J. Reinkensmeyer, ‘Review of control strategies for robotic movement training after neurologic injury,’ Journal of NeuroEngineering and Rehabilitation, vol. 6, article 20, 2009. doi: 10.1186/1743-0003-6-20.",
        "[5] A. A. Blank, J. A. French, A. U. Pehlivan, and M. K. O’Malley, ‘Current trends in robot-assisted upper-limb stroke rehabilitation: Promoting patient engagement in therapy,’ Current Physical Medicine and Rehabilitation Reports, vol. 2, pp. 184-195, 2014. doi: 10.1007/s40141-014-0056-z.",
        "[6] L. E. Kahn, M. L. Zygman, W. Z. Rymer, and D. J. Reinkensmeyer, ‘Robot-assisted reaching exercise promotes arm movement recovery in chronic hemiparetic stroke,’ Journal of NeuroEngineering and Rehabilitation, vol. 3, article 12, 2006. doi: 10.1186/1743-0003-3-12.",
        "[7] I. Aprile, M. Germanotta, A. Cruciani, et al., ‘Upper limb robotic rehabilitation after stroke: A multicenter, randomized clinical trial,’ Journal of Neurologic Physical Therapy, vol. 44, no. 1, pp. 3-14, 2020. doi: 10.1097/NPT.0000000000000295.",
        "[8] S. Y. Jeon, M. Ki, and J.-H. Shin, ‘Resistive versus active assisted robotic training for the upper limb after a stroke,’ Annals of Physical and Rehabilitation Medicine, vol. 67, no. 1, article 101789, 2024. doi: 10.1016/j.rehab.2023.101789.",
        "[9] C. Kim, Y. Moon, and J. Choi, ‘Model predictive control-based assist-as-needed strategy for reducing motor slacking in robot-assisted rehabilitation,’ Sensors, vol. 26, no. 9, article 2740, 2026. doi: 10.3390/s26092740.",
        "[10] Z. Fang, S. Woodford, D. Senanayake, and D. Ackland, ‘Conversion of upper-limb inertial measurement unit data to joint angles: A systematic review,’ Sensors, vol. 23, no. 14, article 6535, 2023. doi: 10.3390/s23146535.",
        "[11] M. El-Gohary and J. McNames, ‘Shoulder and elbow joint angle tracking with inertial sensors,’ IEEE Transactions on Biomedical Engineering, vol. 59, no. 9, pp. 2635-2641, 2012. doi: 10.1109/TBME.2012.2208750.",
        "[12] P. Müller, M.-A. Bégin, T. Schauer, and T. Seel, ‘Alignment-free, self-calibrating elbow angles measurement using inertial sensors,’ IEEE Journal of Biomedical and Health Informatics, vol. 21, no. 2, pp. 312-319, 2017. doi: 10.1109/JBHI.2016.2639537.",
        "[13] M. M. B. Morrow, B. Lowndes, E. Fortune, K. R. Kaufman, and M. S. Hallbeck, ‘Validation of inertial measurement units for upper body kinematics,’ Journal of Applied Biomechanics, vol. 33, no. 3, pp. 227-232, 2017. doi: 10.1123/jab.2016-0120.",
        "[14] F. Karadeniz, Ö. E. Aydoğan, E. A. Kazancı, and E. Akdoğan, ‘Design of a 4-DOF grounded exoskeletal robot for shoulder and elbow rehabilitation,’ Sustainable Engineering and Innovation, vol. 2, no. 1, pp. 41-65, 2020. doi: 10.37868/sei.v2i1.106.",
        "[15] L. Zhang, S. Guo, and Q. Sun, ‘Development and assist-as-needed control of an end-effector upper limb rehabilitation robot,’ Applied Sciences, vol. 10, no. 19, article 6684, 2020. doi: 10.3390/app10196684.",
        "[16] L. Zhang, S. Guo, and F. Xi, ‘Performance-based assistance control for robot-mediated upper-limbs rehabilitation,’ Mechatronics, vol. 89, article 102919, 2023. doi: 10.1016/j.mechatronics.2022.102919.",
        "[17] T. Ahmed, M. R. Islam, B. Brahmi, and M. H. Rahman, ‘Robustness and tracking performance evaluation of PID motion control of 7 DoF anthropomorphic exoskeleton robot assisted upper limb rehabilitation,’ Sensors, vol. 22, no. 10, article 3747, 2022. doi: 10.3390/s22103747.",
        "[18] T. Song, K. Zhang, Z. Yan, Y. Li, S. Guo, and X. Li, ‘Research on upper limb motion intention classification and rehabilitation robot control based on sEMG,’ Sensors, vol. 25, no. 4, article 1057, 2025. doi: 10.3390/s25041057.（未來延伸。）",
        "[19] World Health Organization, ‘Ensuring access to rehabilitation for all,’ 28 May 2024. Available: https://www.who.int/news-room/feature-stories/detail/ensuring-access-to-rehabilitation-for-all",
        "[20] K. Boardsworth, U. Rashid, S. Olsen, et al., ‘Upper limb robotic rehabilitation following stroke: A systematic review and meta-analysis investigating efficacy and the influence of device features and program parameters,’ Journal of NeuroEngineering and Rehabilitation, vol. 22, article 164, 2025. doi: 10.1186/s12984-025-01662-4.",
        "[21] A. U. Pehlivan, D. P. Losey, and M. K. O’Malley, ‘Minimal assist-as-needed controller for upper limb robotic rehabilitation,’ IEEE Transactions on Robotics, vol. 32, no. 1, pp. 113-124, 2016. doi: 10.1109/TRO.2015.2503726.",
        "[22] V. Costa, Ó. Ramírez, A. Otero, D. Muñoz-García, S. Uribarri, and R. Raya, ‘Validity and reliability of inertial sensors for elbow and wrist range of motion assessment,’ PeerJ, vol. 8, article e9687, 2020. doi: 10.7717/peerj.9687.",
        "[23] J. Kaszyński, C. Baka, M. Białecka, and P. Lubiatowski, ‘Shoulder range of motion measurement using inertial measurement unit-concurrent validity and reliability,’ Sensors, vol. 23, no. 17, article 7499, 2023. doi: 10.3390/s23177499.",
        "[24] J. Han, ‘From PID to active disturbance rejection control,’ IEEE Transactions on Industrial Electronics, vol. 56, no. 3, pp. 900-906, 2009. doi: 10.1109/TIE.2008.2011621.",
        "[25] D. A. Bristow, M. Tharayil, and A. G. Alleyne, ‘A survey of iterative learning control: A learning-based method for high-performance tracking control,’ IEEE Control Systems Magazine, vol. 26, no. 3, pp. 96-114, 2006. doi: 10.1109/MCS.2006.1636313.",
        "[26] C. T. Freeman, A.-M. Hughes, J. H. Burridge, P. H. Chappell, P. L. Lewin, and E. Rogers, ‘Iterative learning control of FES applied to the upper extremity for rehabilitation,’ Control Engineering Practice, vol. 17, no. 3, pp. 368-381, 2009. doi: 10.1016/j.conengprac.2008.08.003.",
    ]
    for ref in references:
        p = doc.add_paragraph()
        p.paragraph_format.left_indent = Inches(0.25)
        p.paragraph_format.first_line_indent = Inches(-0.25)
        p.paragraph_format.space_after = Pt(5)
        p.paragraph_format.line_spacing = 1.15
        r = p.add_run(ref)
        set_run_font(r, size=9.5)

    # Re-apply page setup after all content is added.
    configure_page(doc)
    doc.core_properties.title = "基於雙 IMU 回授與目標軌跡導引之上肢復健按需輔助控制系統設計：非臨床工程驗證"
    doc.core_properties.subject = "畢業專題正式研究計畫書"
    doc.core_properties.author = "[學生姓名待填]；指導教授：陳家進"
    doc.core_properties.keywords = "上肢運動量測, 雙IMU, 個人化ROM, Assist-as-Needed, 系統識別, LADRC, ILC, 嵌入式故障反應"
    doc.save(OUT)
    print(OUT)


if __name__ == "__main__":
    build()
