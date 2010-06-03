object formCameraPose: TformCameraPose
  Left = 330
  Top = 234
  Width = 312
  Height = 330
  Caption = 'Change cameras pose and parameters'
  Color = clBtnFace
  Font.Charset = DEFAULT_CHARSET
  Font.Color = clWindowText
  Font.Height = -11
  Font.Name = 'MS Sans Serif'
  Font.Style = []
  OldCreateOrder = False
  PixelsPerInch = 96
  TextHeight = 13
  object rgIndex: TRadioGroup
    Left = 8
    Top = 8
    Width = 233
    Height = 41
    Caption = 'Select observation indices to change: '
    Columns = 5
    ItemIndex = 0
    Items.Strings = (
      '0'
      '1'
      '2'
      '3'
      '4')
    TabOrder = 0
  end
  object PageControl1: TPageControl
    Left = 8
    Top = 56
    Width = 289
    Height = 209
    ActivePage = TabSheet1
    MultiLine = True
    TabOrder = 1
    object TabSheet1: TTabSheet
      Caption = 'Change 3D camera pose'
      object Label1: TLabel
        Left = 32
        Top = 16
        Width = 11
        Height = 13
        Alignment = taRightJustify
        Caption = 'x='
      end
      object Label2: TLabel
        Left = 32
        Top = 40
        Width = 11
        Height = 13
        Alignment = taRightJustify
        Caption = 'y='
      end
      object Label3: TLabel
        Left = 32
        Top = 64
        Width = 11
        Height = 13
        Alignment = taRightJustify
        Caption = 'z='
      end
      object Label4: TLabel
        Left = 154
        Top = 16
        Width = 25
        Height = 13
        Alignment = taRightJustify
        Caption = 'yaw='
      end
      object Label5: TLabel
        Left = 150
        Top = 40
        Width = 29
        Height = 13
        Alignment = taRightJustify
        Caption = 'pitch='
      end
      object Label6: TLabel
        Left = 160
        Top = 64
        Width = 19
        Height = 13
        Alignment = taRightJustify
        Caption = 'roll='
      end
      object Label7: TLabel
        Left = 109
        Top = 16
        Width = 14
        Height = 13
        Alignment = taRightJustify
        Caption = '(m)'
      end
      object Label8: TLabel
        Left = 109
        Top = 40
        Width = 14
        Height = 13
        Alignment = taRightJustify
        Caption = '(m)'
      end
      object Label9: TLabel
        Left = 109
        Top = 64
        Width = 14
        Height = 13
        Alignment = taRightJustify
        Caption = '(m)'
      end
      object Label10: TLabel
        Left = 243
        Top = 16
        Width = 24
        Height = 13
        Alignment = taRightJustify
        Caption = '(deg)'
      end
      object Label11: TLabel
        Left = 243
        Top = 40
        Width = 24
        Height = 13
        Alignment = taRightJustify
        Caption = '(deg)'
      end
      object Label12: TLabel
        Left = 243
        Top = 64
        Width = 24
        Height = 13
        Alignment = taRightJustify
        Caption = '(deg)'
      end
      object Edit1: TEdit
        Left = 48
        Top = 12
        Width = 57
        Height = 21
        TabOrder = 0
        Text = '0.00'
      end
      object Edit2: TEdit
        Left = 48
        Top = 36
        Width = 57
        Height = 21
        TabOrder = 1
        Text = '0.00'
      end
      object Edit3: TEdit
        Left = 48
        Top = 60
        Width = 57
        Height = 21
        TabOrder = 2
        Text = '0.00'
      end
      object Edit4: TEdit
        Left = 184
        Top = 12
        Width = 57
        Height = 21
        TabOrder = 3
        Text = '0.00'
      end
      object Edit5: TEdit
        Left = 184
        Top = 36
        Width = 57
        Height = 21
        TabOrder = 4
        Text = '0.00'
      end
      object Edit6: TEdit
        Left = 184
        Top = 60
        Width = 57
        Height = 21
        TabOrder = 5
        Text = '0.00'
      end
      object Button1: TButton
        Left = 112
        Top = 96
        Width = 73
        Height = 25
        Caption = 'APPLY'
        Font.Charset = DEFAULT_CHARSET
        Font.Color = clWindowText
        Font.Height = -11
        Font.Name = 'MS Sans Serif'
        Font.Style = [fsBold]
        ParentFont = False
        TabOrder = 6
        OnClick = Button1Click
      end
    end
    object TabSheet2: TTabSheet
      Caption = 'Change intrinsic parameters'
      ImageIndex = 1
      object sgMatrix: TStringGrid
        Left = 8
        Top = 8
        Width = 225
        Height = 113
        Hint = 'Enter here the intrinsic parameters matrix'
        ColCount = 3
        DefaultColWidth = 65
        FixedCols = 0
        RowCount = 3
        FixedRows = 0
        Options = [goFixedVertLine, goFixedHorzLine, goVertLine, goHorzLine, goRangeSelect, goColSizing, goEditing, goAlwaysShowEditor]
        ParentShowHint = False
        ShowHint = True
        TabOrder = 0
      end
      object Button2: TButton
        Left = 104
        Top = 128
        Width = 73
        Height = 25
        Caption = 'APPLY'
        Font.Charset = DEFAULT_CHARSET
        Font.Color = clWindowText
        Font.Height = -11
        Font.Name = 'MS Sans Serif'
        Font.Style = [fsBold]
        ParentFont = False
        TabOrder = 1
        OnClick = Button2Click
      end
    end
    object TabSheet3: TTabSheet
      Caption = 'Change distortion parameters'
      ImageIndex = 2
      object sgDistortion: TStringGrid
        Left = 8
        Top = 8
        Width = 225
        Height = 113
        Hint = 'Enter here the intrinsic parameters matrix'
        ColCount = 1
        DefaultColWidth = 80
        FixedCols = 0
        RowCount = 4
        FixedRows = 0
        Options = [goFixedVertLine, goFixedHorzLine, goVertLine, goHorzLine, goRangeSelect, goColSizing, goEditing, goAlwaysShowEditor]
        ParentShowHint = False
        ShowHint = True
        TabOrder = 0
      end
      object Button3: TButton
        Left = 104
        Top = 128
        Width = 73
        Height = 25
        Caption = 'APPLY'
        Font.Charset = DEFAULT_CHARSET
        Font.Color = clWindowText
        Font.Height = -11
        Font.Name = 'MS Sans Serif'
        Font.Style = [fsBold]
        ParentFont = False
        TabOrder = 1
        OnClick = Button3Click
      end
    end
  end
  object BitBtn1: TBitBtn
    Left = 224
    Top = 272
    Width = 75
    Height = 25
    Caption = 'Close'
    TabOrder = 2
    Kind = bkOK
  end
end
