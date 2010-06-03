object Ventana: TVentana
  Left = 223
  Top = 186
  Width = 696
  Height = 520
  Caption = 
    'PTG (Parameterized Trajectories Generator) Designer - @ JLBC - I' +
    '.S.A. 2005 -2007'
  Color = clBtnFace
  Font.Charset = DEFAULT_CHARSET
  Font.Color = clWindowText
  Font.Height = -11
  Font.Name = 'MS Sans Serif'
  Font.Style = []
  OldCreateOrder = False
  ShowHint = True
  WindowState = wsMaximized
  PixelsPerInch = 96
  TextHeight = 13
  object Splitter1: TSplitter
    Left = 241
    Top = 0
    Width = 5
    Height = 493
    Cursor = crHSplit
    Beveled = True
  end
  object Panel1: TPanel
    Left = 0
    Top = 0
    Width = 241
    Height = 493
    Align = alLeft
    BevelOuter = bvNone
    TabOrder = 0
    object PageControl2: TPageControl
      Left = 0
      Top = 120
      Width = 241
      Height = 373
      ActivePage = TabSheet5
      Align = alBottom
      TabOrder = 0
      TabPosition = tpBottom
      OnChange = btnGenerateClick
      object TabSheet5: TTabSheet
        Caption = 'Simulations'
        object Panel2: TPanel
          Left = 0
          Top = 16
          Width = 233
          Height = 329
          Align = alBottom
          BevelOuter = bvNone
          TabOrder = 0
          object Label1: TLabel
            Left = 20
            Top = 8
            Width = 96
            Height = 13
            Alignment = taRightJustify
            Caption = 'Reference distance:'
            Font.Charset = DEFAULT_CHARSET
            Font.Color = clNavy
            Font.Height = -11
            Font.Name = 'MS Sans Serif'
            Font.Style = []
            ParentFont = False
          end
          object Label2: TLabel
            Left = 46
            Top = 32
            Width = 70
            Height = 13
            Alignment = taRightJustify
            Caption = 'Grid resolution:'
            Font.Charset = DEFAULT_CHARSET
            Font.Color = clNavy
            Font.Height = -11
            Font.Name = 'MS Sans Serif'
            Font.Style = []
            ParentFont = False
          end
          object Label4: TLabel
            Left = 31
            Top = 56
            Width = 85
            Height = 13
            Alignment = taRightJustify
            Caption = 'Alfa values count:'
            Font.Charset = DEFAULT_CHARSET
            Font.Color = clNavy
            Font.Height = -11
            Font.Name = 'MS Sans Serif'
            Font.Style = []
            ParentFont = False
          end
          object Label5: TLabel
            Left = 47
            Top = 80
            Width = 69
            Height = 13
            Alignment = taRightJustify
            Caption = 'Max. sim. time:'
            Font.Charset = DEFAULT_CHARSET
            Font.Color = clNavy
            Font.Height = -11
            Font.Name = 'MS Sans Serif'
            Font.Style = []
            ParentFont = False
          end
          object Label6: TLabel
            Left = 50
            Top = 104
            Width = 66
            Height = 13
            Alignment = taRightJustify
            Caption = 'Max. sim. dist:'
            Font.Charset = DEFAULT_CHARSET
            Font.Color = clNavy
            Font.Height = -11
            Font.Name = 'MS Sans Serif'
            Font.Style = []
            ParentFont = False
          end
          object Label7: TLabel
            Left = 41
            Top = 128
            Width = 75
            Height = 13
            Alignment = taRightJustify
            Caption = 'Max. sim. steps:'
            Font.Charset = DEFAULT_CHARSET
            Font.Color = clNavy
            Font.Height = -11
            Font.Name = 'MS Sans Serif'
            Font.Style = []
            ParentFont = False
          end
          object Label8: TLabel
            Left = 80
            Top = 152
            Width = 36
            Height = 13
            Alignment = taRightJustify
            Caption = 'Sim. At:'
            Font.Charset = DEFAULT_CHARSET
            Font.Color = clNavy
            Font.Height = -11
            Font.Name = 'MS Sans Serif'
            Font.Style = []
            ParentFont = False
          end
          object Label9: TLabel
            Left = 26
            Top = 176
            Width = 90
            Height = 13
            Alignment = taRightJustify
            Caption = 'Sim. min stored Ad:'
            Font.Charset = DEFAULT_CHARSET
            Font.Color = clNavy
            Font.Height = -11
            Font.Name = 'MS Sans Serif'
            Font.Style = []
            ParentFont = False
          end
          object Label10: TLabel
            Left = 77
            Top = 200
            Width = 39
            Height = 13
            Alignment = taRightJustify
            Caption = 'MAX. V:'
            Font.Charset = DEFAULT_CHARSET
            Font.Color = clNavy
            Font.Height = -11
            Font.Name = 'MS Sans Serif'
            Font.Style = []
            ParentFont = False
          end
          object Label11: TLabel
            Left = 73
            Top = 224
            Width = 43
            Height = 13
            Alignment = taRightJustify
            Caption = 'MAX. W:'
            Font.Charset = DEFAULT_CHARSET
            Font.Color = clNavy
            Font.Height = -11
            Font.Name = 'MS Sans Serif'
            Font.Style = []
            ParentFont = False
          end
          object Label17: TLabel
            Left = 91
            Top = 248
            Width = 25
            Height = 13
            Alignment = taRightJustify
            Caption = 'TAU:'
            Font.Charset = DEFAULT_CHARSET
            Font.Color = clNavy
            Font.Height = -11
            Font.Name = 'MS Sans Serif'
            Font.Style = []
            ParentFont = False
          end
          object Label18: TLabel
            Left = 78
            Top = 272
            Width = 38
            Height = 13
            Alignment = taRightJustify
            Caption = 'DELAY:'
            Font.Charset = DEFAULT_CHARSET
            Font.Color = clNavy
            Font.Height = -11
            Font.Name = 'MS Sans Serif'
            Font.Style = []
            ParentFont = False
          end
          object btnGenerate: TButton
            Left = 8
            Top = 296
            Width = 217
            Height = 33
            Caption = 'PERFORM SIMULATIONS...'
            Default = True
            Font.Charset = DEFAULT_CHARSET
            Font.Color = clWindowText
            Font.Height = -11
            Font.Name = 'MS Sans Serif'
            Font.Style = [fsBold]
            ParentFont = False
            TabOrder = 0
            OnClick = btnGenerateClick
          end
          object edRefDist: TEdit
            Left = 128
            Top = 4
            Width = 73
            Height = 21
            TabOrder = 1
            Text = '3.5'
          end
          object edRes: TEdit
            Left = 128
            Top = 28
            Width = 73
            Height = 21
            TabOrder = 2
            Text = '0.02'
          end
          object edAlfas: TEdit
            Left = 128
            Top = 52
            Width = 73
            Height = 21
            TabOrder = 3
            Text = '200'
          end
          object edMaxTime: TEdit
            Left = 128
            Top = 76
            Width = 73
            Height = 21
            TabOrder = 4
            Text = '60'
          end
          object edMaxDist: TEdit
            Left = 128
            Top = 100
            Width = 73
            Height = 21
            TabOrder = 5
            Text = '3.5'
          end
          object edMaxN: TEdit
            Left = 128
            Top = 124
            Width = 73
            Height = 21
            TabOrder = 6
            Text = '500'
          end
          object edAT: TEdit
            Left = 128
            Top = 148
            Width = 73
            Height = 21
            TabOrder = 7
            Text = '0.01'
          end
          object edMinDist: TEdit
            Left = 128
            Top = 172
            Width = 73
            Height = 21
            TabOrder = 8
            Text = '0.015'
          end
          object edVM: TEdit
            Left = 128
            Top = 196
            Width = 73
            Height = 21
            Hint = 'Max. linear speed (m/s)'
            TabOrder = 9
            Text = '0.4'
          end
          object edWM: TEdit
            Left = 128
            Top = 220
            Width = 73
            Height = 21
            Hint = 'Max. angular speed (deg/s)'
            TabOrder = 10
            Text = '30'
          end
          object edTAU: TEdit
            Left = 128
            Top = 244
            Width = 73
            Height = 21
            Hint = 'Max. linear speed (m/s)'
            TabOrder = 11
            Text = '0'
          end
          object edDELAY: TEdit
            Left = 128
            Top = 268
            Width = 73
            Height = 21
            Hint = 'Max. linear speed (m/s)'
            TabOrder = 12
            Text = '0'
          end
        end
      end
      object TabSheet6: TTabSheet
        Caption = 'Operations'
        ImageIndex = 1
        object Label15: TLabel
          Left = 16
          Top = 8
          Width = 23
          Height = 13
          Caption = 'alfa='
        end
        object Label16: TLabel
          Left = 16
          Top = 40
          Width = 33
          Height = 13
          Caption = 'Cursor:'
        end
        object lbCursor1: TLabel
          Left = 24
          Top = 56
          Width = 9
          Height = 13
          Caption = '---'
        end
        object lbCursor2: TLabel
          Left = 24
          Top = 72
          Width = 12
          Height = 13
          Caption = '----'
        end
        object edSelAlfa: TEdit
          Left = 42
          Top = 4
          Width = 55
          Height = 21
          TabOrder = 0
          Text = '0'
        end
        object Button1: TButton
          Left = 100
          Top = 2
          Width = 125
          Height = 25
          Caption = 'Highlight trajectory...'
          TabOrder = 1
          OnClick = Button1Click
        end
        object Button2: TButton
          Left = 8
          Top = 312
          Width = 217
          Height = 25
          Caption = 'Save C-Space surface for MATLAB...'
          TabOrder = 2
          OnClick = Button2Click
        end
        object Button3: TButton
          Left = 8
          Top = 280
          Width = 217
          Height = 25
          Caption = 'Save function V( a )...'
          TabOrder = 3
          OnClick = Button3Click
        end
      end
    end
    object Panel3: TPanel
      Left = 0
      Top = 80
      Width = 241
      Height = 40
      Align = alBottom
      TabOrder = 1
      object Label13: TLabel
        Left = 79
        Top = 8
        Width = 21
        Height = 13
        Alignment = taRightJustify
        Caption = 'a0v:'
        Font.Charset = DEFAULT_CHARSET
        Font.Color = clNavy
        Font.Height = -11
        Font.Name = 'MS Sans Serif'
        Font.Style = []
        ParentFont = False
      end
      object Label14: TLabel
        Left = 157
        Top = 8
        Width = 23
        Height = 13
        Alignment = taRightJustify
        Caption = 'a0w:'
        Font.Charset = DEFAULT_CHARSET
        Font.Color = clNavy
        Font.Height = -11
        Font.Name = 'MS Sans Serif'
        Font.Style = []
        ParentFont = False
      end
      object Label3: TLabel
        Left = 5
        Top = 8
        Width = 7
        Height = 13
        Alignment = taRightJustify
        Caption = 'K'
        Font.Charset = DEFAULT_CHARSET
        Font.Color = clNavy
        Font.Height = -11
        Font.Name = 'MS Sans Serif'
        Font.Style = []
        ParentFont = False
      end
      object edA0V: TEdit
        Left = 112
        Top = 4
        Width = 33
        Height = 21
        TabOrder = 0
        Text = '15'
      end
      object edA0W: TEdit
        Left = 192
        Top = 4
        Width = 33
        Height = 21
        TabOrder = 1
        Text = '90'
      end
      object edK: TEdit
        Left = 24
        Top = 4
        Width = 41
        Height = 21
        TabOrder = 2
        Text = '1.0'
      end
    end
    object rgPTG: TListBox
      Left = 0
      Top = 0
      Width = 241
      Height = 80
      Align = alClient
      ItemHeight = 13
      Items.Strings = (
        '#1: C'
        '#2: a-A'
        '#3: C|CS'
        '#4: C|C'
        '#5: CS'
        '#6: Sp-S'
        '#7: CCS')
      TabOrder = 2
      OnClick = btnGenerateClick
    end
  end
  object PageControl1: TPageControl
    Left = 246
    Top = 0
    Width = 442
    Height = 493
    ActivePage = TabSheet1
    Align = alClient
    TabOrder = 1
    object TabSheet1: TTabSheet
      Caption = 'View in WS'
      object plot: Tsp_XYPlot
        Left = 0
        Top = 0
        Width = 434
        Height = 400
        Align = alClient
        Color = clBtnFace
        ParentColor = False
        TabOrder = 0
        OnMouseMove = plotMouseMove
        LeftAxis.Margin = 4
        LeftAxis.LineAttr.Color = clBlack
        LeftAxis.LineAttr.Visible = True
        LeftAxis.GridAttr.Color = clGray
        LeftAxis.GridAttr.Visible = True
        LeftAxis.LabelFormat = '###0.##'
        LeftAxis.SFlags = 193
        LeftAxis.SLinePos = (
          27
          378
          373)
        LeftAxis.fMin = 0.775
        LeftAxis.fMax = 10.225
        RightAxis.Margin = 4
        RightAxis.LineAttr.Color = clBlack
        RightAxis.LineAttr.Visible = True
        RightAxis.GridAttr.Color = clGray
        RightAxis.GridAttr.Visible = False
        RightAxis.LabelFormat = '###0.##'
        RightAxis.SFlags = 57
        RightAxis.SLinePos = (
          429
          378
          373)
        RightAxis.fMax = 10
        BottomAxis.Margin = 4
        BottomAxis.LineAttr.Color = clBlack
        BottomAxis.LineAttr.Visible = True
        BottomAxis.GridAttr.Color = clGray
        BottomAxis.GridAttr.Visible = True
        BottomAxis.LabelFormat = '###0.##'
        BottomAxis.SFlags = 192
        BottomAxis.SLinePos = (
          28
          379
          400)
        BottomAxis.fMin = -0.225
        BottomAxis.fMax = 9.225
        TopAxis.Margin = 4
        TopAxis.LineAttr.Color = clBlack
        TopAxis.LineAttr.Visible = True
        TopAxis.GridAttr.Color = clGray
        TopAxis.GridAttr.Visible = False
        TopAxis.LabelFormat = '###0.##'
        TopAxis.SFlags = 56
        TopAxis.SLinePos = (
          28
          4
          400)
        TopAxis.fMax = 10
        BorderStyle = bs_None
        FieldColor = clWhite
      end
      object memOut: TMemo
        Left = 0
        Top = 400
        Width = 434
        Height = 65
        Align = alBottom
        Color = clBtnFace
        ReadOnly = True
        ScrollBars = ssBoth
        TabOrder = 1
      end
    end
    object TabSheet7: TTabSheet
      Caption = 'Design functions'
      ImageIndex = 2
      object plotW: Tsp_XYPlot
        Left = 0
        Top = 209
        Width = 434
        Height = 209
        Align = alTop
        Color = clBtnFace
        ParentColor = False
        TabOrder = 0
        OnMouseMove = plotMouseMove
        LeftAxis.Margin = 4
        LeftAxis.LineAttr.Color = clBlack
        LeftAxis.LineAttr.Visible = True
        LeftAxis.GridAttr.Color = clGray
        LeftAxis.GridAttr.Visible = True
        LeftAxis.LabelFormat = '###0.##'
        LeftAxis.SFlags = 193
        LeftAxis.SLinePos = (
          27
          187
          182)
        LeftAxis.fMin = 1.8
        LeftAxis.fMax = 10.2
        RightAxis.Margin = 4
        RightAxis.LineAttr.Color = clBlack
        RightAxis.LineAttr.Visible = True
        RightAxis.GridAttr.Color = clGray
        RightAxis.GridAttr.Visible = False
        RightAxis.LabelFormat = '###0.##'
        RightAxis.SFlags = 57
        RightAxis.SLinePos = (
          429
          187
          182)
        RightAxis.fMax = 10
        BottomAxis.Margin = 4
        BottomAxis.LineAttr.Color = clBlack
        BottomAxis.LineAttr.Visible = True
        BottomAxis.GridAttr.Color = clGray
        BottomAxis.GridAttr.Visible = True
        BottomAxis.LabelFormat = '###0.##'
        BottomAxis.SFlags = 192
        BottomAxis.SLinePos = (
          28
          188
          400)
        BottomAxis.fMin = -0.225
        BottomAxis.fMax = 9.225
        TopAxis.Margin = 4
        TopAxis.LineAttr.Color = clBlack
        TopAxis.LineAttr.Visible = True
        TopAxis.GridAttr.Color = clGray
        TopAxis.GridAttr.Visible = False
        TopAxis.LabelFormat = '###0.##'
        TopAxis.SFlags = 56
        TopAxis.SLinePos = (
          28
          4
          400)
        TopAxis.fMax = 10
        BorderStyle = bs_None
        FieldColor = clWhite
      end
      object plotV: Tsp_XYPlot
        Left = 0
        Top = 0
        Width = 434
        Height = 209
        Align = alTop
        Color = clBtnFace
        ParentColor = False
        TabOrder = 1
        OnMouseMove = plotMouseMove
        LeftAxis.Margin = 4
        LeftAxis.LineAttr.Color = clBlack
        LeftAxis.LineAttr.Visible = True
        LeftAxis.GridAttr.Color = clGray
        LeftAxis.GridAttr.Visible = True
        LeftAxis.LabelFormat = '###0.##'
        LeftAxis.SFlags = 193
        LeftAxis.SLinePos = (
          27
          187
          182)
        LeftAxis.fMin = 0.775
        LeftAxis.fMax = 10.225
        RightAxis.Margin = 4
        RightAxis.LineAttr.Color = clBlack
        RightAxis.LineAttr.Visible = True
        RightAxis.GridAttr.Color = clGray
        RightAxis.GridAttr.Visible = False
        RightAxis.LabelFormat = '###0.##'
        RightAxis.SFlags = 57
        RightAxis.SLinePos = (
          429
          187
          182)
        RightAxis.fMax = 10
        BottomAxis.Margin = 4
        BottomAxis.LineAttr.Color = clBlack
        BottomAxis.LineAttr.Visible = True
        BottomAxis.GridAttr.Color = clGray
        BottomAxis.GridAttr.Visible = True
        BottomAxis.LabelFormat = '###0.##'
        BottomAxis.SFlags = 192
        BottomAxis.SLinePos = (
          28
          188
          400)
        BottomAxis.fMin = -0.225
        BottomAxis.fMax = 9.225
        TopAxis.Margin = 4
        TopAxis.LineAttr.Color = clBlack
        TopAxis.LineAttr.Visible = True
        TopAxis.GridAttr.Color = clGray
        TopAxis.GridAttr.Visible = False
        TopAxis.LabelFormat = '###0.##'
        TopAxis.SFlags = 56
        TopAxis.SLinePos = (
          28
          4
          400)
        TopAxis.fMax = 10
        BorderStyle = bs_None
        FieldColor = clWhite
      end
    end
    object TabSheet4: TTabSheet
      Caption = 'View in 3D C-Space'
      ImageIndex = 1
      object sbH: TScrollBar
        Left = 0
        Top = 449
        Width = 434
        Height = 16
        Align = alBottom
        Max = 180
        Min = -180
        PageSize = 0
        TabOrder = 1
      end
      object sbV: TScrollBar
        Left = 418
        Top = 0
        Width = 16
        Height = 449
        Align = alRight
        Kind = sbVertical
        Max = 180
        Min = -180
        PageSize = 0
        TabOrder = 0
      end
    end
    object TabSheet2: TTabSheet
      Caption = '"Target control" viewpoint'
      ImageIndex = 3
      object plotTarget: Tsp_XYPlot
        Left = 0
        Top = 0
        Width = 434
        Height = 465
        Align = alClient
        Color = clBtnFace
        ParentColor = False
        TabOrder = 0
        OnMouseMove = plotMouseMove
        LeftAxis.Margin = 4
        LeftAxis.LineAttr.Color = clBlack
        LeftAxis.LineAttr.Visible = True
        LeftAxis.GridAttr.Color = clGray
        LeftAxis.GridAttr.Visible = True
        LeftAxis.LabelFormat = '###0.##'
        LeftAxis.SFlags = 193
        LeftAxis.SLinePos = (
          27
          443
          438)
        LeftAxis.fMin = 0.775
        LeftAxis.fMax = 10.225
        RightAxis.Margin = 4
        RightAxis.LineAttr.Color = clBlack
        RightAxis.LineAttr.Visible = True
        RightAxis.GridAttr.Color = clGray
        RightAxis.GridAttr.Visible = False
        RightAxis.LabelFormat = '###0.##'
        RightAxis.SFlags = 57
        RightAxis.SLinePos = (
          429
          443
          438)
        RightAxis.fMax = 10
        BottomAxis.Margin = 4
        BottomAxis.LineAttr.Color = clBlack
        BottomAxis.LineAttr.Visible = True
        BottomAxis.GridAttr.Color = clGray
        BottomAxis.GridAttr.Visible = True
        BottomAxis.LabelFormat = '###0.##'
        BottomAxis.SFlags = 192
        BottomAxis.SLinePos = (
          28
          444
          400)
        BottomAxis.fMin = -0.225
        BottomAxis.fMax = 9.225
        TopAxis.Margin = 4
        TopAxis.LineAttr.Color = clBlack
        TopAxis.LineAttr.Visible = True
        TopAxis.GridAttr.Color = clGray
        TopAxis.GridAttr.Visible = False
        TopAxis.LabelFormat = '###0.##'
        TopAxis.SFlags = 56
        TopAxis.SLinePos = (
          28
          4
          400)
        TopAxis.fMax = 10
        BorderStyle = bs_None
        FieldColor = clWhite
      end
    end
  end
  object pTrajs: Tsp_XYLine
    Plot = plot
    LineAttr.Color = clNavy
    LineAttr.Visible = True
    PointAttr.Color = clBlue
    PointAttr.Kind = ptEllipse
    PointAttr.HSize = 3
    PointAttr.VSize = 3
    PointAttr.Visible = False
    PointAttr.BorderWidth = 0
    PointAttr.BorderColor = clBlue
    Left = 258
    Top = 24
  end
  object pSelTraj: Tsp_XYLine
    Plot = plot
    LineAttr.Color = clRed
    LineAttr.Width = 2
    LineAttr.Visible = True
    PointAttr.Color = clBlue
    PointAttr.Kind = ptEllipse
    PointAttr.HSize = 3
    PointAttr.VSize = 3
    PointAttr.Visible = False
    PointAttr.BorderWidth = 0
    PointAttr.BorderColor = clBlue
    Left = 258
    Top = 56
  end
  object SD: TSaveDialog
    DefaultExt = 'txt'
    FileName = 'x_y_phi.txt'
    Filter = '*.txt|*.txt'
    InitialDir = '.'
    Options = [ofOverwritePrompt, ofHideReadOnly, ofEnableSizing]
    Left = 20
    Top = 358
  end
  object pV: Tsp_XYLine
    Plot = plotV
    LineAttr.Color = clNavy
    LineAttr.Visible = True
    PointAttr.Color = clBlue
    PointAttr.Kind = ptEllipse
    PointAttr.HSize = 3
    PointAttr.VSize = 3
    PointAttr.Visible = False
    PointAttr.BorderWidth = 0
    PointAttr.BorderColor = clBlue
    Left = 290
    Top = 24
  end
  object pW: Tsp_XYLine
    Plot = plotW
    LineAttr.Color = clNavy
    LineAttr.Visible = True
    PointAttr.Color = clBlue
    PointAttr.Kind = ptEllipse
    PointAttr.HSize = 3
    PointAttr.VSize = 3
    PointAttr.Visible = False
    PointAttr.BorderWidth = 0
    PointAttr.BorderColor = clBlue
    Left = 322
    Top = 24
  end
  object trajsTarget: Tsp_XYLine
    Plot = plotTarget
    LineAttr.Color = clNavy
    LineAttr.Visible = True
    PointAttr.Color = clBlue
    PointAttr.Kind = ptEllipse
    PointAttr.HSize = 3
    PointAttr.VSize = 3
    PointAttr.Visible = False
    PointAttr.BorderWidth = 0
    PointAttr.BorderColor = clBlue
    Left = 258
    Top = 88
  end
end
