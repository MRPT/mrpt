object formAligner: TformAligner
  Left = 299
  Top = 90
  BorderStyle = bsDialog
  Caption = 'Alignment parameters'
  ClientHeight = 529
  ClientWidth = 867
  Color = clBtnFace
  Font.Charset = DEFAULT_CHARSET
  Font.Color = clWindowText
  Font.Height = -11
  Font.Name = 'MS Sans Serif'
  Font.Style = []
  OldCreateOrder = False
  Position = poScreenCenter
  PixelsPerInch = 96
  TextHeight = 13
  object Label1: TLabel
    Left = 24
    Top = 16
    Width = 139
    Height = 13
    Caption = 'Reference observation index:'
  end
  object Label2: TLabel
    Left = 240
    Top = 16
    Width = 105
    Height = 13
    Caption = '(The index in the map)'
  end
  object Label16: TLabel
    Left = 72
    Top = 40
    Width = 89
    Height = 13
    Caption = 'Obs. to be aligned:'
  end
  object btnCompute: TBitBtn
    Left = 256
    Top = 240
    Width = 113
    Height = 25
    Caption = 'COMPUTE'
    Font.Charset = DEFAULT_CHARSET
    Font.Color = clWindowText
    Font.Height = -11
    Font.Name = 'MS Sans Serif'
    Font.Style = [fsBold]
    ParentFont = False
    TabOrder = 0
    OnClick = btnComputeClick
    Glyph.Data = {
      DE010000424DDE01000000000000760000002800000024000000120000000100
      0400000000006801000000000000000000001000000000000000000000000000
      80000080000000808000800000008000800080800000C0C0C000808080000000
      FF0000FF000000FFFF00FF000000FF00FF00FFFF0000FFFFFF00333333333333
      3333333333333333333333330000333333333333333333333333F33333333333
      00003333344333333333333333388F3333333333000033334224333333333333
      338338F3333333330000333422224333333333333833338F3333333300003342
      222224333333333383333338F3333333000034222A22224333333338F338F333
      8F33333300003222A3A2224333333338F3838F338F33333300003A2A333A2224
      33333338F83338F338F33333000033A33333A222433333338333338F338F3333
      0000333333333A222433333333333338F338F33300003333333333A222433333
      333333338F338F33000033333333333A222433333333333338F338F300003333
      33333333A222433333333333338F338F00003333333333333A22433333333333
      3338F38F000033333333333333A223333333333333338F830000333333333333
      333A333333333333333338330000333333333333333333333333333333333333
      0000}
    NumGlyphs = 2
  end
  object edRefIndex: TEdit
    Left = 168
    Top = 12
    Width = 57
    Height = 21
    TabOrder = 1
    Text = '0'
  end
  object rgMethod: TRadioGroup
    Left = 8
    Top = 120
    Width = 305
    Height = 57
    Caption = ' Method to be used: '
    Columns = 2
    ItemIndex = 0
    Items.Strings = (
      'ICP vs. Points map'
      'ICP vs. Grid map')
    TabOrder = 2
  end
  object PageControl1: TPageControl
    Left = 0
    Top = 271
    Width = 369
    Height = 177
    ActivePage = TabSheet1
    TabIndex = 0
    TabOrder = 3
    object TabSheet1: TTabSheet
      Caption = 'ICP params.'
      object Label3: TLabel
        Left = 90
        Top = 16
        Width = 45
        Height = 13
        Alignment = taRightJustify
        Caption = 'Max. dist:'
      end
      object Label4: TLabel
        Left = 64
        Top = 72
        Width = 71
        Height = 13
        Alignment = taRightJustify
        Caption = 'Max. iterations:'
      end
      object Label17: TLabel
        Left = 93
        Top = 40
        Width = 42
        Height = 13
        Alignment = taRightJustify
        Caption = 'Min. dist.'
      end
      object Label18: TLabel
        Left = 242
        Top = 16
        Width = 21
        Height = 13
        Alignment = taRightJustify
        Caption = 'Alfa:'
      end
      object edMaxDist: TEdit
        Left = 144
        Top = 12
        Width = 65
        Height = 21
        TabOrder = 0
        Text = '0.7'
      end
      object edMaxIters: TEdit
        Left = 144
        Top = 68
        Width = 65
        Height = 21
        TabOrder = 1
        Text = '35'
      end
      object edSmallestDist: TEdit
        Left = 144
        Top = 36
        Width = 65
        Height = 21
        TabOrder = 2
        Text = '0.3'
      end
      object Button1: TButton
        Left = 224
        Top = 72
        Width = 97
        Height = 25
        Caption = 'Animate...'
        TabOrder = 3
        OnClick = Button1Click
      end
      object edAlfa: TEdit
        Left = 272
        Top = 12
        Width = 65
        Height = 21
        TabOrder = 4
        Text = '0.5'
      end
    end
    object TabSheet2: TTabSheet
      Caption = 'Correlation'
      ImageIndex = 1
      object Label5: TLabel
        Left = 24
        Top = 16
        Width = 143
        Height = 13
        Alignment = taRightJustify
        Caption = 'Max. dist. for correspondence:'
      end
      object Label6: TLabel
        Left = 92
        Top = 40
        Width = 75
        Height = 13
        Alignment = taRightJustify
        Caption = 'Distance range:'
      end
      object Label7: TLabel
        Left = 98
        Top = 64
        Width = 69
        Height = 13
        Alignment = taRightJustify
        Caption = 'Angular range:'
      end
      object Label8: TLabel
        Left = 125
        Top = 88
        Width = 42
        Height = 13
        Alignment = taRightJustify
        Caption = 'Step XY:'
      end
      object Label9: TLabel
        Left = 121
        Top = 112
        Width = 46
        Height = 13
        Alignment = taRightJustify
        Caption = 'Step PHI:'
      end
      object Label10: TLabel
        Left = 247
        Top = 64
        Width = 24
        Height = 13
        Alignment = taRightJustify
        Caption = '(deg)'
      end
      object Label11: TLabel
        Left = 247
        Top = 112
        Width = 24
        Height = 13
        Alignment = taRightJustify
        Caption = '(deg)'
      end
      object edMaxDistCorr: TEdit
        Left = 176
        Top = 12
        Width = 65
        Height = 21
        TabOrder = 0
        Text = '0'
      end
      object edRangeXY: TEdit
        Left = 176
        Top = 36
        Width = 65
        Height = 21
        TabOrder = 1
        Text = '1.0'
      end
      object edRangePHI: TEdit
        Left = 176
        Top = 60
        Width = 65
        Height = 21
        TabOrder = 2
        Text = '15.0'
      end
      object edStepXY: TEdit
        Left = 176
        Top = 84
        Width = 65
        Height = 21
        TabOrder = 3
        Text = '0.10'
      end
      object edStepPHI: TEdit
        Left = 176
        Top = 108
        Width = 65
        Height = 21
        TabOrder = 4
        Text = '1.0'
      end
    end
  end
  object GroupBox1: TGroupBox
    Left = 8
    Top = 184
    Width = 305
    Height = 49
    Caption = 'Initial pose: '
    TabOrder = 4
    object Label12: TLabel
      Left = 8
      Top = 20
      Width = 11
      Height = 13
      Caption = 'x='
    end
    object Label13: TLabel
      Left = 88
      Top = 20
      Width = 11
      Height = 13
      Caption = 'y='
    end
    object Label14: TLabel
      Left = 160
      Top = 20
      Width = 20
      Height = 13
      Caption = 'phi='
    end
    object Label15: TLabel
      Left = 240
      Top = 20
      Width = 24
      Height = 13
      Caption = '(deg)'
    end
    object edX: TEdit
      Left = 24
      Top = 16
      Width = 49
      Height = 21
      TabOrder = 0
      Text = '0'
    end
    object edY: TEdit
      Left = 104
      Top = 16
      Width = 49
      Height = 21
      TabOrder = 1
      Text = '0'
    end
    object edPhi: TEdit
      Left = 184
      Top = 16
      Width = 49
      Height = 21
      TabOrder = 2
      Text = '0'
    end
  end
  object edIndex: TEdit
    Left = 168
    Top = 36
    Width = 57
    Height = 21
    TabOrder = 5
    Text = '1'
  end
  object plot: Tsp_XYPlot
    Left = 392
    Top = 0
    Width = 465
    Height = 441
    Color = clBtnFace
    ParentColor = False
    TabOrder = 6
    LeftAxis.Margin = 4
    LeftAxis.LineAttr.Color = clBlack
    LeftAxis.LineAttr.Visible = True
    LeftAxis.GridAttr.Color = clGray
    LeftAxis.GridAttr.Visible = True
    LeftAxis.LabelFormat = '###0.##'
    LeftAxis.SFlags = 1
    LeftAxis.SLinePos = (
      27
      419
      410)
    LeftAxis.fMax = 10
    RightAxis.Margin = 4
    RightAxis.LineAttr.Color = clBlack
    RightAxis.LineAttr.Visible = True
    RightAxis.GridAttr.Color = clGray
    RightAxis.GridAttr.Visible = False
    RightAxis.LabelFormat = '###0.##'
    RightAxis.SFlags = 57
    RightAxis.SLinePos = (
      455
      419
      410)
    RightAxis.fMax = 10
    BottomAxis.Margin = 4
    BottomAxis.LineAttr.Color = clBlack
    BottomAxis.LineAttr.Visible = True
    BottomAxis.GridAttr.Color = clGray
    BottomAxis.GridAttr.Visible = True
    BottomAxis.LabelFormat = '###0.##'
    BottomAxis.SFlags = 0
    BottomAxis.SLinePos = (
      28
      420
      426)
    BottomAxis.fMax = 10
    TopAxis.Margin = 4
    TopAxis.LineAttr.Color = clBlack
    TopAxis.LineAttr.Visible = True
    TopAxis.GridAttr.Color = clGray
    TopAxis.GridAttr.Visible = False
    TopAxis.LabelFormat = '###0.##'
    TopAxis.SFlags = 56
    TopAxis.SLinePos = (
      28
      8
      426)
    TopAxis.fMax = 10
    BorderStyle = bs_None
    FieldColor = clWhite
  end
  object log: TMemo
    Left = 0
    Top = 456
    Width = 867
    Height = 73
    Align = alBottom
    Font.Charset = DEFAULT_CHARSET
    Font.Color = clWindowText
    Font.Height = -11
    Font.Name = 'Courier'
    Font.Style = []
    ParentFont = False
    ReadOnly = True
    ScrollBars = ssBoth
    TabOrder = 7
  end
  object p1: Tsp_XYLine
    Plot = plot
    LineAttr.Color = clBlack
    LineAttr.Visible = False
    PointAttr.Color = clBlue
    PointAttr.Kind = ptRectangle
    PointAttr.HSize = 3
    PointAttr.VSize = 3
    PointAttr.Visible = True
    PointAttr.BorderWidth = 0
    PointAttr.BorderColor = clBlue
    Left = 464
    Top = 24
  end
  object p2: Tsp_XYLine
    Plot = plot
    LineAttr.Color = clBlack
    LineAttr.Visible = False
    PointAttr.Color = clRed
    PointAttr.Kind = ptRectangle
    PointAttr.HSize = 3
    PointAttr.VSize = 3
    PointAttr.Visible = True
    PointAttr.BorderColor = clRed
    Left = 504
    Top = 24
  end
end
