class UsersController < ApplicationController
  def list_users
  	render json: {users: [{name: 'Michael'}, {name: 'Quan'}]}
  end
end
