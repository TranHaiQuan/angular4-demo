Rails.application.routes.draw do
  root 'pages#index'
  scope "/api" do
  	
  end

  get "*unmatchedroute", to: "pages#index"
  # get 'users', to: 'users#list_users'

  # For details on the DSL available within this file, see http://guides.rubyonrails.org/routing.html
end
