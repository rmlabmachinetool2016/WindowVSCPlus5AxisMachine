#pragma once

namespace fiveaxis_machine_tool {

	using namespace System;
	using namespace System::ComponentModel;
	using namespace System::Collections;
	using namespace System::Windows::Forms;
	using namespace System::Data;
	using namespace System::Drawing;

	/// <summary>
	/// Form2 ÇÃäTóv
	/// </summary>
	public ref class Form2 : public System::Windows::Forms::Form
	{
	public:
		Form2(void)
		{
			InitializeComponent();
			//
			//TODO: Ç±Ç±Ç…ÉRÉìÉXÉgÉâÉNÉ^Å[ ÉRÅ[ÉhÇí«â¡ÇµÇ‹Ç∑
			//
			cap_max_x0 = cap_max_x1 = width;
			cap_max_x2 = cap_max_x3 = width;
			cap_max_x4 = cap_max_x5 = width;
			cap_max_x6 = cap_max_x7 = width;
			cap_max_x8 = cap_max_x9 = width;
			cap_max_x10 = width;

			cap_min_x0 = cap_min_x1 = 0;
			cap_min_x2 = cap_min_x3 = 0;
			cap_min_x4 = cap_min_x5 = 0;
			cap_min_x6 = cap_min_x7 = 0;
			cap_min_x8 = cap_min_x9 = 0;
			cap_min_x10 = 0;

			cap_max_y0 = cap_max_y1 = hight;
			cap_max_y2 = cap_max_y3 = hight;
			cap_max_y4 = cap_max_y5 = hight;
			cap_max_y6 = cap_max_y7 = hight;
			cap_max_y8 = cap_max_y9 = hight;
			cap_max_y10 = hight;

			cap_min_y0 = cap_min_y1 = 0;
			cap_min_y2 = cap_min_y3 = 0;
			cap_min_y4 = cap_min_y5 = 0;
			cap_min_y6 = cap_min_y7 = 0;
			cap_min_y8 = cap_min_y9 = 0;
			cap_min_y10 = 0;
		}

	protected:
		/// <summary>
		/// égópíÜÇÃÉäÉ\Å[ÉXÇÇ∑Ç◊ÇƒÉNÉäÅ[ÉìÉAÉbÉvÇµÇ‹Ç∑ÅB
		/// </summary>
		~Form2()
		{
			if (components)
			{
				delete components;
			}
		}

	protected: 

	private:
		/// <summary>
		/// ïKóvÇ»ÉfÉUÉCÉiÅ[ïœêîÇ≈Ç∑ÅB
		/// </summary>
		System::ComponentModel::Container ^components;

#pragma region Windows Form Designer generated code
		/// <summary>
		/// ÉfÉUÉCÉiÅ[ ÉTÉ|Å[ÉgÇ…ïKóvÇ»ÉÅÉ\ÉbÉhÇ≈Ç∑ÅBÇ±ÇÃÉÅÉ\ÉbÉhÇÃì‡óeÇ
		/// ÉRÅ[Éh ÉGÉfÉBÉ^Å[Ç≈ïœçXÇµÇ»Ç¢Ç≈Ç≠ÇæÇ≥Ç¢ÅB
		/// </summary>
		void InitializeComponent(void)
		{
			this->SuspendLayout();
			// 
			// Form2
			// 
			this->AutoScaleDimensions = System::Drawing::SizeF(6, 12);
			this->AutoScaleMode = System::Windows::Forms::AutoScaleMode::Font;
			this->ClientSize = System::Drawing::Size(852, 446);
			this->Name = L"Form2";
			this->Text = L"Graphs";
			this->Paint += gcnew System::Windows::Forms::PaintEventHandler(this, &Form2::Form2_Paint);
			this->ResumeLayout(false);

		}
#pragma endregion
	private: System::Void Form2_Paint(System::Object^  sender, System::Windows::Forms::PaintEventArgs^  e) {
				 Graphics^ g = e->Graphics;

				 DrawGraph1(g);
				 DrawGraph2(g);
				 DrawGraph3(g);
				 DrawGraph4(g);
				 DrawGraph5(g);
				 DrawGraph6(g);
				 DrawGraph7(g);
				 DrawGraph8(g);
				 DrawGraph9(g);
				 DrawGraph10(g);
				 DrawGraph11(g);
				 DrawGraph12(g);
				 DrawGraph13(g);
			 }
	private: void DrawGraph1( Graphics^ g  ){
				 //length = graph1->GetLength(1);
				 length = num_data;
				 x = gcnew array<int>(length);
				 y = gcnew array<int>(length);
			 
				 max_val_y = min_val_y = graph1[1,0];
				 for(i=1;i<length;i++)
				 {
					 if(max_val_y < graph1[1,i])
						 max_val_y = graph1[1,i];
					 if(min_val_y > graph1[1,i])
						 min_val_y = graph1[1,i];
				 }
				 convert_x = width / graph1[0,length-1];
				 convert_y = -hight / (max_val_y - min_val_y);
				 for(i=0;i<length;i++)
				 {
					 x[i] = static_cast<int>(graph1[0,i]*convert_x + x0);
					 y[i] = static_cast<int>(graph1[1,i]*convert_y + hight/2 + y0);
				 }
				 xlabel_max = graph1[0,length-1];
				 xlabel_min = graph1[0,0];
				 ylabel_max = max_val_y;
				 ylabel_min = min_val_y;

				 g->FillRectangle(Brushes::White,Rectangle(x0,y0,width,hight));		// îwåiÇÃìhÇËÇ¬Ç‘Çµ
				 g->DrawLine(Pens::LightGray,x0,hight/2+y0,x0+width,hight/2+y0);	// é≤ÇÃï`âÊ
				 for(i=0;i<length-1;i++)
				 {
					 if(y[i]<y0 ||y[i+1]<y0 || y[i]>y0+hight || y[i+1]>y0+hight)// ÇÕÇ›èoÇ∑ï™ÇÕï`âÊÇµÇ»Ç¢
						 continue;
					 g->DrawLine(Pens::Red,x[i],y[i],x[i+1],y[i+1]);
				 }

				 g->DrawString("contouring error profile",Font,Brushes::Black,static_cast<float>(x0),0);	// caption of graph1
			 }
	private: void DrawGraph2( Graphics^ g  ){
				 length = num_data;
				 x = gcnew array<int>(length);
				 y = gcnew array<int>(length);
			 
				 max_val_y = min_val_y = graph2[1,0];
				 for(i=1;i<length;i++)
				 {
					 if(max_val_y < graph2[1,i])
						 max_val_y = graph2[1,i];
					 if(min_val_y > graph2[1,i])
						 min_val_y = graph2[1,i];
				 }
				 convert_x = width / graph2[0,length-1];
				 convert_y = -hight / (max_val_y - min_val_y);
				 for(i=0;i<length;i++)
				 {
					 x[i] = static_cast<int>(graph2[0,i]*convert_x + x1);
					 y[i] = static_cast<int>(graph2[1,i]*convert_y + hight/2 + y1);
				 }
				 xlabel_max = graph2[0,length-1];
				 xlabel_min = graph2[0,0];
				 ylabel_max = max_val_y;
				 ylabel_min = min_val_y;

				 g->FillRectangle(Brushes::White,Rectangle(x1,y1,width,hight));		// îwåiÇÃìhÇËÇ¬Ç‘Çµ
				 g->DrawLine(Pens::LightGray,x1,hight/2+y1,x1+width,hight/2+y1);	// é≤ÇÃï`âÊ
				 for(i=0;i<length-1;i++)
				 {
					 if(y[i]<y1 ||y[i+1]<y1 || y[i]>y1+hight || y[i+1]>y1+hight)// ÇÕÇ›èoÇ∑ï™ÇÕï`âÊÇµÇ»Ç¢
						 continue;
					 g->DrawLine(Pens::Red,x[i],y[i],x[i+1],y[i+1]);
				 }

				 g->DrawString("ec vs en profile",Font,Brushes::Black,static_cast<float>(x1),static_cast<float>(y0+hight));	// caption of graph2
			 }
	private: void DrawGraph3( Graphics^ g  ){
				 length = num_data;
				 x = gcnew array<int>(length);
				 y = gcnew array<int>(length);
			 
				 max_val_y = min_val_y = graph3[1,0];
				 for(i=1;i<length;i++)
				 {
					 if(max_val_y < graph3[1,i])
						 max_val_y = graph3[1,i];
					 if(min_val_y > graph3[1,i])
						 min_val_y = graph3[1,i];
				 }
				 convert_x = width / graph3[0,length-1];
				 convert_y = -hight / (max_val_y - min_val_y);
				 for(i=0;i<length;i++)
				 {
					 x[i] = static_cast<int>(graph3[0,i]*convert_x + x2);
					 y[i] = static_cast<int>(graph3[1,i]*convert_y + hight/2 + y2);
				 }
				 xlabel_max = graph3[0,length-1];
				 xlabel_min = graph3[0,0];
				 ylabel_max = max_val_y;
				 ylabel_min = min_val_y;

				 g->FillRectangle(Brushes::White,Rectangle(x2,y2,width,hight));		// îwåiÇÃìhÇËÇ¬Ç‘Çµ
				 g->DrawLine(Pens::LightGray,x2,hight/2+y2,x2+width,hight/2+y2);	// é≤ÇÃï`âÊ
				 for(i=0;i<length-1;i++)
				 {
					 if(y[i]<y2 ||y[i+1]<y2 || y[i]>y2+hight || y[i+1]>y2+hight)// ÇÕÇ›èoÇ∑ï™ÇÕï`âÊÇµÇ»Ç¢
						 continue;
					 g->DrawLine(Pens::Red,x[i],y[i],x[i+1],y[i+1]);
				 }

				 g->DrawString("ec vs en(mod) profile",Font,Brushes::Black,static_cast<float>(x2),static_cast<float>(y1+hight));	// caption of graph3
			 }
	private: void DrawGraph4( Graphics^ g  ){
				 length = num_data;
				 x = gcnew array<int>(length);
				 y = gcnew array<int>(length);
			 
				 max_val_y = min_val_y = graph4[1,0];
				 for(i=1;i<length;i++)
				 {
					 if(max_val_y < graph4[1,i])
						 max_val_y = graph4[1,i];
					 if(min_val_y > graph4[1,i])
						 min_val_y = graph4[1,i];
				 }
				 convert_x = width / graph4[0,length-1];
				 convert_y = -hight / (max_val_y - min_val_y);
				 for(i=0;i<length;i++)
				 {
					 x[i] = static_cast<int>(graph4[0,i]*convert_x + x3);
					 y[i] = static_cast<int>(graph4[1,i]*convert_y + hight/2 + y3);
				 }
				 xlabel_max = graph4[0,length-1];
				 xlabel_min = graph4[0,0];
				 ylabel_max = max_val_y;
				 ylabel_min = min_val_y;

				 g->FillRectangle(Brushes::White,Rectangle(x3,y3,width,hight));		// îwåiÇÃìhÇËÇ¬Ç‘Çµ
				 g->DrawLine(Pens::LightGray,x3,hight/2+y3,x3+width,hight/2+y3);	// é≤ÇÃï`âÊ
				 for(i=0;i<length-1;i++)
				 {
					 if(y[i]<y3 ||y[i+1]<y3 || y[i]>y3+hight || y[i+1]>y3+hight)// ÇÕÇ›èoÇ∑ï™ÇÕï`âÊÇµÇ»Ç¢
						 continue;
					 g->DrawLine(Pens::Red,x[i],y[i],x[i+1],y[i+1]);
				 }

				 g->DrawString("force(x-axis) profile",Font,Brushes::Black,static_cast<float>(x3),static_cast<float>(0));	// caption of graph4
			 }
	private: void DrawGraph5( Graphics^ g  ){
				 length = num_data;
				 x = gcnew array<int>(length);
				 y = gcnew array<int>(length);
			 
				 max_val_y = min_val_y = graph5[1,0];
				 for(i=1;i<length;i++)
				 {
					 if(max_val_y < graph5[1,i])
						 max_val_y = graph5[1,i];
					 if(min_val_y > graph5[1,i])
						 min_val_y = graph5[1,i];
				 }
				 convert_x = width / graph5[0,length-1];
				 convert_y = -hight / (max_val_y - min_val_y);
				 for(i=0;i<length;i++)
				 {
					 x[i] = static_cast<int>(graph5[0,i]*convert_x + x4);
					 y[i] = static_cast<int>(graph5[1,i]*convert_y + hight/2 + y4);
				 }
				 xlabel_max = graph5[0,length-1];
				 xlabel_min = graph5[0,0];
				 ylabel_max = max_val_y;
				 ylabel_min = min_val_y;

				 g->FillRectangle(Brushes::White,Rectangle(x4,y4,width,hight));		// îwåiÇÃìhÇËÇ¬Ç‘Çµ
				 g->DrawLine(Pens::LightGray,x4,hight/2+y4,x4+width,hight/2+y4);	// é≤ÇÃï`âÊ
				 for(i=0;i<length-1;i++)
				 {
					 if(y[i]<y4 ||y[i+1]<y4 || y[i]>y4+hight || y[i+1]>y4+hight)// ÇÕÇ›èoÇ∑ï™ÇÕï`âÊÇµÇ»Ç¢
						 continue;
					 g->DrawLine(Pens::Red,x[i],y[i],x[i+1],y[i+1]);
				 }

				 g->DrawString("force(y1-axis) profile",Font,Brushes::Black,static_cast<float>(x4),static_cast<float>(y3+hight));	// caption of graph5
			 }
	private: void DrawGraph6( Graphics^ g  ){
				 length = num_data;
				 x = gcnew array<int>(length);
				 y = gcnew array<int>(length);
			 
				 max_val_y = min_val_y = graph6[1,0];
				 for(i=1;i<length;i++)
				 {
					 if(max_val_y < graph6[1,i])
						 max_val_y = graph6[1,i];
					 if(min_val_y > graph6[1,i])
						 min_val_y = graph6[1,i];
				 }
				 convert_x = width / graph6[0,length-1];
				 convert_y = -hight / (max_val_y - min_val_y);
				 for(i=0;i<length;i++)
				 {
					 x[i] = static_cast<int>(graph6[0,i]*convert_x + x5);
					 y[i] = static_cast<int>(graph6[1,i]*convert_y + hight/2 + y5);
				 }
				 xlabel_max = graph6[0,length-1];
				 xlabel_min = graph6[0,0];
				 ylabel_max = max_val_y;
				 ylabel_min = min_val_y;

				 g->FillRectangle(Brushes::White,Rectangle(x5,y5,width,hight));		// îwåiÇÃìhÇËÇ¬Ç‘Çµ
				 g->DrawLine(Pens::LightGray,x5,hight/2+y5,x5+width,hight/2+y5);	// é≤ÇÃï`âÊ
				 for(i=0;i<length-1;i++)
				 {
					 if(y[i]<y5 ||y[i+1]<y5 || y[i]>y5+hight || y[i+1]>y5+hight)// ÇÕÇ›èoÇ∑ï™ÇÕï`âÊÇµÇ»Ç¢
						 continue;
					 g->DrawLine(Pens::Red,x[i],y[i],x[i+1],y[i+1]);
				 }

				 g->DrawString("force(y2-axis) profile",Font,Brushes::Black,static_cast<float>(x5),static_cast<float>(y4+hight));	// caption of graph6
			 }
	private: void DrawGraph7( Graphics^ g  ){
				 length = num_data;
				 x = gcnew array<int>(length);
				 y = gcnew array<int>(length);
			 
				 max_val_y = min_val_y = graph7[1,0];
				 for(i=1;i<length;i++)
				 {
					 if(max_val_y < graph7[1,i])
						 max_val_y = graph7[1,i];
					 if(min_val_y > graph7[1,i])
						 min_val_y = graph7[1,i];
				 }
				 convert_x = width / graph7[0,length-1];
				 convert_y = -hight / (max_val_y - min_val_y);
				 for(i=0;i<length;i++)
				 {
					 x[i] = static_cast<int>(graph7[0,i]*convert_x + x6);
					 y[i] = static_cast<int>(graph7[1,i]*convert_y + hight/2 + y6);
				 }
				 xlabel_max = graph7[0,length-1];
				 xlabel_min = graph7[0,0];
				 ylabel_max = max_val_y;
				 ylabel_min = min_val_y;

				 g->FillRectangle(Brushes::White,Rectangle(x6,y6,width,hight));		// îwåiÇÃìhÇËÇ¬Ç‘Çµ
				 g->DrawLine(Pens::LightGray,x6,hight/2+y6,x6+width,hight/2+y6);	// é≤ÇÃï`âÊ
				 for(i=0;i<length-1;i++)
				 {
					 if(y[i]<y6 ||y[i+1]<y6 || y[i]>y6+hight || y[i+1]>y6+hight)// ÇÕÇ›èoÇ∑ï™ÇÕï`âÊÇµÇ»Ç¢
						 continue;
					 g->DrawLine(Pens::Red,x[i],y[i],x[i+1],y[i+1]);
				 }

				 g->DrawString("force(z-axis) profile",Font,Brushes::Black,static_cast<float>(x6),static_cast<float>(y5+hight));	// caption of graph7
			 }
	private: void DrawGraph8( Graphics^ g  ){
				 length = num_data;
				 x = gcnew array<int>(length);
				 y = gcnew array<int>(length);
			 
				 max_val_y = min_val_y = graph8[1,0];
				 for(i=1;i<length;i++)
				 {
					 if(max_val_y < graph8[1,i])
						 max_val_y = graph8[1,i];
					 if(min_val_y > graph8[1,i])
						 min_val_y = graph8[1,i];
				 }
				 convert_x = width / graph8[0,length-1];
				 convert_y = -hight / (max_val_y - min_val_y);
				 for(i=0;i<length;i++)
				 {
					 x[i] = static_cast<int>(graph8[0,i]*convert_x + x7);
					 y[i] = static_cast<int>(graph8[1,i]*convert_y + hight/2 + y7);
				 }
				 xlabel_max = graph8[0,length-1];
				 xlabel_min = graph8[0,0];
				 ylabel_max = max_val_y;
				 ylabel_min = min_val_y;

				 g->FillRectangle(Brushes::White,Rectangle(x7,y7,width,hight));		// îwåiÇÃìhÇËÇ¬Ç‘Çµ
				 g->DrawLine(Pens::LightGray,x7,hight/2+y7,x7+width,hight/2+y7);	// é≤ÇÃï`âÊ
				 for(i=0;i<length-1;i++)
				 {
					 if(y[i]<y7 ||y[i+1]<y7 || y[i]>y7+hight || y[i+1]>y7+hight)// ÇÕÇ›èoÇ∑ï™ÇÕï`âÊÇµÇ»Ç¢
						 continue;
					 g->DrawLine(Pens::Red,x[i],y[i],x[i+1],y[i+1]);
				 }

				 g->DrawString("torque(c-axis) profile",Font,Brushes::Black,static_cast<float>(x7),static_cast<float>(0));	// caption of graph8
			 }
	private: void DrawGraph9( Graphics^ g  ){
				 length = num_data;
				 x = gcnew array<int>(length);
				 y = gcnew array<int>(length);
			 
				 max_val_y = min_val_y = graph9[1,0];
				 for(i=1;i<length;i++)
				 {
					 if(max_val_y < graph9[1,i])
						 max_val_y = graph9[1,i];
					 if(min_val_y > graph9[1,i])
						 min_val_y = graph9[1,i];
				 }
				 convert_x = width / graph9[0,length-1];
				 convert_y = -hight / (max_val_y - min_val_y);
				 for(i=0;i<length;i++)
				 {
					 x[i] = static_cast<int>(graph9[0,i]*convert_x + x8);
					 y[i] = static_cast<int>(graph9[1,i]*convert_y + hight/2 + y8);
				 }
				 xlabel_max = graph9[0,length-1];
				 xlabel_min = graph9[0,0];
				 ylabel_max = max_val_y;
				 ylabel_min = min_val_y;

				 g->FillRectangle(Brushes::White,Rectangle(x8,y8,width,hight));		// îwåiÇÃìhÇËÇ¬Ç‘Çµ
				 g->DrawLine(Pens::LightGray,x8,hight/2+y8,x8+width,hight/2+y8);	// é≤ÇÃï`âÊ
				 for(i=0;i<length-1;i++)
				 {
					 if(y[i]<y8 ||y[i+1]<y8 || y[i]>y8+hight || y[i+1]>y8+hight)// ÇÕÇ›èoÇ∑ï™ÇÕï`âÊÇµÇ»Ç¢
						 continue;
					 g->DrawLine(Pens::Red,x[i],y[i],x[i+1],y[i+1]);
				 }

				 g->DrawString("torque(a1-axis) profile",Font,Brushes::Black,static_cast<float>(x8),static_cast<float>(y7+hight));	// caption of graph9
			 }
	private: void DrawGraph10( Graphics^ g  ){
				 length = num_data;
				 x = gcnew array<int>(length);
				 y = gcnew array<int>(length);
			 
				 max_val_y = min_val_y = graph10[1,0];
				 for(i=1;i<length;i++)
				 {
					 if(max_val_y < graph10[1,i])
						 max_val_y = graph10[1,i];
					 if(min_val_y > graph10[1,i])
						 min_val_y = graph10[1,i];
				 }
				 convert_x = width / graph10[0,length-1];
				 convert_y = -hight / (max_val_y - min_val_y);
				 for(i=0;i<length;i++)
				 {
					 x[i] = static_cast<int>(graph10[0,i]*convert_x + x9);
					 y[i] = static_cast<int>(graph10[1,i]*convert_y + hight/2 + y9);
				 }
				 xlabel_max = graph10[0,length-1];
				 xlabel_min = graph10[0,0];
				 ylabel_max = max_val_y;
				 ylabel_min = min_val_y;

				 g->FillRectangle(Brushes::White,Rectangle(x9,y9,width,hight));		// îwåiÇÃìhÇËÇ¬Ç‘Çµ
				 g->DrawLine(Pens::LightGray,x9,hight/2+y9,x9+width,hight/2+y9);	// é≤ÇÃï`âÊ
				 for(i=0;i<length-1;i++)
				 {
					 if(y[i]<y9 ||y[i+1]<y9 || y[i]>y9+hight || y[i+1]>y9+hight)// ÇÕÇ›èoÇ∑ï™ÇÕï`âÊÇµÇ»Ç¢
						 continue;
					 g->DrawLine(Pens::Red,x[i],y[i],x[i+1],y[i+1]);
				 }

				 g->DrawString("torque(a2-axis) profile",Font,Brushes::Black,static_cast<float>(x9),static_cast<float>(y8+hight));	// caption of graph10
			 }
	private: void DrawGraph11( Graphics^ g  ){
				 length = num_data;
				 x = gcnew array<int>(length);
				 y = gcnew array<int>(length);
			 
				 max_val_y = min_val_y = graph11[1,0];
				 for(i=1;i<length;i++)
				 {
					 if(max_val_y < graph11[1,i])
						 max_val_y = graph11[1,i];
					 if(min_val_y > graph11[1,i])
						 min_val_y = graph11[1,i];
				 }
				 convert_x = width / graph11[0,length-1];
				 convert_y = -hight / (max_val_y - min_val_y);
				 for(i=0;i<length;i++)
				 {
					 x[i] = static_cast<int>(graph11[0,i]*convert_x + x10);
					 y[i] = static_cast<int>(graph11[1,i]*convert_y + hight/2 + y10);
				 }
				 xlabel_max = graph11[0,length-1];
				 xlabel_min = graph11[0,0];
				 ylabel_max = max_val_y;
				 ylabel_min = min_val_y;

				 g->FillRectangle(Brushes::White,Rectangle(x10,y10,width,hight));		// îwåiÇÃìhÇËÇ¬Ç‘Çµ
				 g->DrawLine(Pens::LightGray,x10,hight/2+y10,x10+width,hight/2+y10);	// é≤ÇÃï`âÊ
				 for(i=0;i<length-1;i++)
				 {
					 if(y[i]<y10 ||y[i+1]<y10 || y[i]>y10+hight || y[i+1]>y10+hight)// ÇÕÇ›èoÇ∑ï™ÇÕï`âÊÇµÇ»Ç¢
						 continue;
					 g->DrawLine(Pens::Red,x[i],y[i],x[i+1],y[i+1]);
				 }

				 g->DrawString("tangetial error profile",Font,Brushes::Black,static_cast<float>(x10),0.0);	// caption of graph11
			 }
	private: void DrawGraph12( Graphics^ g  ){
				 length = num_data;
				 x = gcnew array<int>(length);
				 y = gcnew array<int>(length);
			 
				 max_val_y = min_val_y = graph12[1,0];
				 for(i=1;i<length;i++)
				 {
					 if(max_val_y < graph12[1,i])
						 max_val_y = graph12[1,i];
					 if(min_val_y > graph12[1,i])
						 min_val_y = graph12[1,i];
				 }
				 convert_x = width / graph12[0,length-1];
				 convert_y = -hight / (max_val_y - min_val_y);
				 for(i=0;i<length;i++)
				 {
					 x[i] = static_cast<int>(graph12[0,i]*convert_x + x11);
					 y[i] = static_cast<int>(graph12[1,i]*convert_y + hight/2 + y11);
				 }
				 xlabel_max = graph12[0,length-1];
				 xlabel_min = graph12[0,0];
				 ylabel_max = max_val_y;
				 ylabel_min = min_val_y;

				 g->FillRectangle(Brushes::White,Rectangle(x11,y11,width,hight));		// îwåiÇÃìhÇËÇ¬Ç‘Çµ
				 g->DrawLine(Pens::LightGray,x11,hight/2+y11,x11+width,hight/2+y11);	// é≤ÇÃï`âÊ
				 for(i=0;i<length-1;i++)
				 {
					 if(y[i]<y11 ||y[i+1]<y11 || y[i]>y11+hight || y[i+1]>y11+hight)// ÇÕÇ›èoÇ∑ï™ÇÕï`âÊÇµÇ»Ç¢
						 continue;
					 g->DrawLine(Pens::Red,x[i],y[i],x[i+1],y[i+1]);
				 }

				 g->DrawString("normal error profile",Font,Brushes::Black,static_cast<float>(x11),static_cast<float>(y10+hight));	// caption of graph12
			 }
	private: void DrawGraph13( Graphics^ g  ){
				 length = num_data;
				 x = gcnew array<int>(length);
				 y = gcnew array<int>(length);
			 
				 max_val_y = min_val_y = graph13[1,0];
				 for(i=1;i<length;i++)
				 {
					 if(max_val_y < graph13[1,i])
						 max_val_y = graph13[1,i];
					 if(min_val_y > graph13[1,i])
						 min_val_y = graph13[1,i];
				 }
				 convert_x = width / graph13[0,length-1];
				 convert_y = -hight / (max_val_y - min_val_y);
				 for(i=0;i<length;i++)
				 {
					 x[i] = static_cast<int>(graph13[0,i]*convert_x + x12);
					 y[i] = static_cast<int>(graph13[1,i]*convert_y + hight/2 + y12);
				 }
				 xlabel_max = graph13[0,length-1];
				 xlabel_min = graph13[0,0];
				 ylabel_max = max_val_y;
				 ylabel_min = min_val_y;

				 g->FillRectangle(Brushes::White,Rectangle(x12,y12,width,hight));		// îwåiÇÃìhÇËÇ¬Ç‘Çµ
				 g->DrawLine(Pens::LightGray,x12,hight/2+y12,x12+width,hight/2+y12);	// é≤ÇÃï`âÊ
				 for(i=0;i<length-1;i++)
				 {
					 if(y[i]<y12 ||y[i+1]<y12 || y[i]>y12+hight || y[i+1]>y12+hight)// ÇÕÇ›èoÇ∑ï™ÇÕï`âÊÇµÇ»Ç¢
						 continue;
					 g->DrawLine(Pens::Red,x[i],y[i],x[i+1],y[i+1]);
				 }

				 g->DrawString("binormal error profile",Font,Brushes::Black,static_cast<float>(x12),static_cast<float>(y11+hight));	// caption of graph13
			 }
	private:
		array<int>^ x,^y;
		double convert_x,convert_y,max_val_y,min_val_y;
		int i,j;
		int length;
		double xlabel_max,xlabel_min,ylabel_max,ylabel_min;
	public:
		static const int hight = 100,width = 200;	// hight and width of graph
		static const int x0 = 10,			y0 = 10,
						 x1 = x0,			y1 = y0+hight+10,
						 x2 = x0,			y2 = y1+hight+10,

						 x3 = x0+width+10,	y3 = y0,
						 x4 = x3,			y4 = y1,
						 x5 = x3,			y5 = y2,
						 x6 = x3,			y6 = y2+hight+10,

						 x7 = x3+width+10,	y7 = y3,
						 x8 = x7,			y8 = y4,
						 x9 = x7,			y9 = y5,

						 x10 = x7+width+10,	y10 = y7,
						 x11 = x10,			y11 = y8,
						 x12 = x10,			y12 = y9;
		double cap_max_x0,cap_min_x0,
			   cap_max_x1,cap_min_x1,
			   cap_max_x2,cap_min_x2,
			   cap_max_x3,cap_min_x3,
			   cap_max_x4,cap_min_x4,
			   cap_max_x5,cap_min_x5,
			   cap_max_x6,cap_min_x6,
			   cap_max_x7,cap_min_x7,
			   cap_max_x8,cap_min_x8,
			   cap_max_x9,cap_min_x9,
			   cap_max_x10,cap_min_x10,
			   cap_max_x11,cap_min_x11,
			   cap_max_x12,cap_min_x12,

			   cap_max_y0,cap_min_y0,
			   cap_max_y1,cap_min_y1,
			   cap_max_y2,cap_min_y2,
			   cap_max_y3,cap_min_y3,
			   cap_max_y4,cap_min_y4,
			   cap_max_y5,cap_min_y5,
			   cap_max_y6,cap_min_y6,
			   cap_max_y7,cap_min_y7,
			   cap_max_y8,cap_min_y8,
			   cap_max_y9,cap_min_y9,
			   cap_max_y10,cap_min_y10,
			   cap_max_y11,cap_min_y11,
			   cap_max_y12,cap_min_y12;
		array<double,2>^ graph1,^ graph2,^ graph3,^ graph4,^ graph5,
					   ^ graph6,^ graph7,^ graph8,^ graph9,^ graph10,
					   ^ graph11,^ graph12,^ graph13;
		unsigned int num_data;
	};
}
